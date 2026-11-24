import { Rng } from '../../math/rng';
import { add, sub, vec3, type Vec3 } from '../../math/vec3';
import { PRESETS, type DeltaParams } from '../../kinematics/params';
import { RobotController, type TaskProvider } from '../../robot/controller';
import type { Strategy, Target } from '../../scheduling/scheduler';
import type { Metric, Scenario } from '../scenario';

export type PlaceMode = 'trayConveyor' | 'staticTrays';
export type TrayFlow = 'co' | 'counter';
export type ProductMix = 'boxes' | 'mixed';

export interface PickPlaceConfig {
  seed: number;
  strategy: Strategy;
  conveyorSpeed: number;
  spawnRate: number;
  placeMode: PlaceMode;
  trayFlow: TrayFlow;
  traySpeed: number;
  traySync: boolean;
  mix: ProductMix;
  alignProducts: boolean;
  spacing: number;
}

export const DEFAULT_PICK_PLACE: PickPlaceConfig = {
  seed: 42,
  strategy: 'edf',
  conveyorSpeed: 0.3,
  spawnRate: 1.2,
  placeMode: 'trayConveyor',
  trayFlow: 'counter',
  traySpeed: 0.15,
  traySync: true,
  mix: 'mixed',
  alignProducts: true,
  spacing: 1.5,
};

export const LAYOUT = {
  beltHeight: 0.8,
  baseHeight: 1.8,
  belt: { y: -0.27, width: 0.44, xStart: -2.4, xEnd: 2.4 },
  trayBelt: { y: 0.3, width: 0.38, xStart: -2.4, xEnd: 2.4 },
  tray: { size: 0.3, height: 0.04, pitch: 0.42, slots: 2 },
  staticTrays: [vec3(-0.25, 0.32, 0), vec3(0.15, 0.32, 0)],
  missX: 0.9,
} as const;

export type ProductKind = 'box' | 'cylinder';
export const PRODUCT_SIZE: Record<ProductKind, { x: number; y: number; h: number }> = {
  box: { x: 0.12, y: 0.08, h: 0.05 },
  cylinder: { x: 0.04, y: 0.04, h: 0.12 },
};

export interface Product {
  id: number;
  kind: ProductKind;
  color: number;
  pos: Vec3;
  yaw: number;
  state: 'belt' | 'reserved' | 'carried' | 'placed' | 'missed';
  trayId?: number;
}

export interface Tray {
  id: number;
  pos: Vec3;
  slots: (number | null)[];
  static: boolean;
}

const SLOT_ID_BASE = 1_000_000;
export const TOOL_LENGTH = { suction: 0.1, fingers: 0.12 };

export class PickPlaceScenario implements Scenario, TaskProvider {
  readonly id = 'pickPlace' as const;
  readonly params: DeltaParams = PRESETS.picker;
  
  readonly robots: RobotController[] = [];
  
  readonly products: Product[] = [];
  readonly trays: Tray[] = [];
  time = 0;
  readonly finished = false;

  private rng: Rng;
  private nextId = 1;
  private nextSpawn = 0;
  private nextTrayId = 1;
  private placed = 0;
  private missed = 0;
  private traysDone = 0;
  private traysIncomplete = 0;
  private waitingForScroll = false;
  beltTravel = 0;
  trayTravel = 0;

  constructor(public config: PickPlaceConfig = DEFAULT_PICK_PLACE) {
    this.rng = new Rng(config.seed);
    
    const makeRobot = () => {
      const pickZ = -(LAYOUT.baseHeight - LAYOUT.beltHeight) + PRODUCT_SIZE.box.h;
      const r = new RobotController( // <-- Исправлена синтаксическая ошибка
        {
          params: this.params,
          tool: TOOL_LENGTH.suction,
          arch: {
            travelZ: pickZ + 0.07,
            minLift: 0.04,
            xy: { vMax: 5, aMax: 60, jMax: 3000 },
            z: { vMax: 3, aMax: 60, jMax: 3000 },
            overlap: 0.7,
          },
          strategy: config.strategy,
          graspTime: 0.08,
          releaseTime: 0.06,
          home: vec3(0, 0.05, pickZ + 0.1),
          margin: 0.05,
          maxInterceptTime: 3,
        },
        this,
      );
      r.id = this.robots.length; // 0 или 1
      return r;
    };
    
    this.robots.push(makeRobot());
    this.robots.push(makeRobot());
    
    this.setupTrays();
    for (let t = 0; t < 6; t += 1 / 30) this.advanceWorld(1 / 30, false);
  }

  configure(patch: Partial<PickPlaceConfig>) {
    this.config = { ...this.config, ...patch };
    for (const r of this.robots) r.config.strategy = this.config.strategy;
  }

  robotBase(index: number = 0): Vec3 {
    const offsetX = index === 0 ? -0.8 : 0.8; 
    return vec3(offsetX, 0, LAYOUT.baseHeight);
  }

  get traySpeed(): number {
    if (!this.config.traySync) return this.config.traySpeed;
    const slots = LAYOUT.tray.slots ** 2;
    return (this.config.spawnRate * LAYOUT.tray.pitch * 1.05) / slots;
  }

  private get trayVelocity(): number {
    if (this.config.placeMode === 'staticTrays') return 0;
    return this.config.trayFlow === 'co' ? this.traySpeed : -this.traySpeed;
  }

  private newTray(pos: Vec3, isStatic: boolean): Tray {
    const n = LAYOUT.tray.slots * LAYOUT.tray.slots;
    return { id: this.nextTrayId++, pos, slots: Array(n).fill(null), static: isStatic };
  }

  private setupTrays() {
    if (this.config.placeMode === 'staticTrays') {
      for (const p of LAYOUT.staticTrays)
        this.trays.push(this.newTray(vec3(p.x, p.y, LAYOUT.beltHeight), true));
      return;
    }
    // Создаем 4 лотка (2 ряда по 2) в зоне роботов
    const { y } = LAYOUT.trayBelt;
    const startX = -0.4;
    for (let i = 0; i < 2; i++) {
      for (let j = 0; j < 2; j++) {
        const x = startX + i * LAYOUT.tray.pitch;
        const yy = y + (j - 0.5) * LAYOUT.tray.pitch;
        this.trays.push(this.newTray(vec3(x, yy, LAYOUT.beltHeight), true));
      }
    }
  }

  private spawnProduct() {
    const kind: ProductKind = 'cylinder'; // Пока только сардельки
    const y = LAYOUT.belt.y;
    const pos = vec3(LAYOUT.belt.xStart, y, LAYOUT.beltHeight);
    
    this.products.push({
      id: this.nextId++,
      kind,
      color: 0xf97316,
      pos,
      yaw: 0,
      state: 'belt',
    });
    return true;
  }

  private advanceWorld(dt: number, withRobot: boolean) {
    const v = this.config.conveyorSpeed;
    this.beltTravel += v * dt;

    // Спавн паровозиком
    const timeToTravelSpacing = this.config.spacing / v;
    this.nextSpawn -= dt;
    if (this.nextSpawn <= 0) {
      this.spawnProduct();
      this.nextSpawn = timeToTravelSpacing;
    }

    for (const p of this.products) {
      if (p.state === 'belt' || p.state === 'reserved' || p.state === 'missed')
        p.pos = vec3(p.pos.x + v * dt, p.pos.y, p.pos.z);
      if (p.state === 'belt' && p.pos.x > LAYOUT.missX && withRobot) {
        p.state = 'missed';
        this.missed++;
      }
    }

    // Лотки стоят на месте
    for (const p of this.products) {
      if (p.state !== 'placed') continue;
      const tray = this.trays.find((t) => t.id === p.trayId);
      if (tray && !tray.static) p.pos = vec3(p.pos.x, p.pos.y, p.pos.z);
    }

    for (let i = this.products.length - 1; i >= 0; i--) {
      const p = this.products[i]!;
      if ((p.state === 'belt' || p.state === 'missed') && p.pos.x > LAYOUT.belt.xEnd) {
        if (p.state === 'belt' && withRobot) this.missed++;
        this.products.splice(i, 1);
      }
    }
  }

  step(dt: number) {
    this.time += dt;
    this.advanceWorld(dt, true);
    
    this.robots.forEach((r, i) => {
      r.base = this.robotBase(i);
      r.step(this.time, dt);
    });
    
    this.robots.forEach((r) => {
      const carried = r.carrying;
      if (carried) {
        const p = this.products.find((q) => q.id === carried.id);
        if (p) {
          const eff = add(r.base, r.state.p);
          p.pos = vec3(eff.x, eff.y, eff.z - PRODUCT_SIZE[p.kind].h);
          p.yaw = r.state.yaw - r.carryYawOffset;
        }
      }
    });

    // Логика термоформера: если все 4 лотка заполнены, прокручиваем
    if (!this.waitingForScroll) {
      const allFull = this.trays.every(t => t.slots.every(s => s !== null));
      if (allFull && this.trays.length > 0) {
        this.waitingForScroll = true;
        setTimeout(() => {
          for (const tray of this.trays) {
            tray.pos = vec3(tray.pos.x + LAYOUT.tray.pitch * 2, tray.pos.y, tray.pos.z);
            tray.slots.fill(null);
          }
          this.waitingForScroll = false;
          this.traysDone++;
        }, 1000);
      }
    }
  }

  private toRobot(world: Vec3): Vec3 {
    return sub(world, this.robotBase(0));
  }

  private productTarget(p: Product): Target {
    return {
      id: p.id,
      pos: this.toRobot(vec3(p.pos.x, p.pos.y, p.pos.z + PRODUCT_SIZE[p.kind].h)),
      vel: vec3(this.config.conveyorSpeed, 0, 0),
      yaw: p.yaw,
      symmetry: p.kind === 'box' ? Math.PI : 0,
    };
  }

  slotPosition(tray: Tray, k: number): Vec3 {
    const n = LAYOUT.tray.slots;
    const pitch = LAYOUT.tray.size / n;
    const ix = k % n;
    const iy = Math.floor(k / n);
    return vec3(
      tray.pos.x + (ix - (n - 1) / 2) * pitch,
      tray.pos.y + (iy - (n - 1) / 2) * pitch,
      tray.pos.z + LAYOUT.tray.height,
    );
  }

  private slotTarget(tray: Tray, k: number, itemHeight: number): Target {
    const s = this.slotPosition(tray, k);
    return {
      id: SLOT_ID_BASE + tray.id * 16 + k,
      pos: this.toRobot(vec3(s.x, s.y, s.z + itemHeight + 0.005)),
      vel: vec3(0, 0, 0),
      yaw: this.config.alignProducts ? 0 : Number.NaN,
      symmetry: Math.PI,
    };
  }

  pickCandidates(robotId: number = 0): Target[] {
    const all = this.products.filter((p) => p.state === 'belt').map((p) => this.productTarget(p));
    
    return all.filter(t => {
      if (robotId === 0) {
        return t.pos.x < 0.4;
      } else {
        return t.pos.x >= -0.4;
      }
    });
  }

  placeCandidates(item: Target): Target[] {
    const product = this.products.find((p) => p.id === item.id);
    const h = product ? PRODUCT_SIZE[product.kind].h : 0.05;
    const out: Target[] = [];
    for (const tray of this.trays) {
      tray.slots.forEach((s, k) => {
        if (s === null) {
          const t = this.slotTarget(tray, k, h);
          out.push(Number.isNaN(t.yaw) ? { ...t, yaw: item.yaw } : t);
        }
      });
    }
    return out;
  }

  find(id: number): Target | undefined {
    if (id >= SLOT_ID_BASE) {
      const trayId = Math.floor((id - SLOT_ID_BASE) / 16);
      const k = (id - SLOT_ID_BASE) % 16;
      const tray = this.trays.find((t) => t.id === trayId);
      if (!tray) return undefined;
      const carried = this.robots.find(r => r.carrying)?.carrying;
      const product = carried ? this.products.find(p => p.id === carried.id) : undefined;
      const t = this.slotTarget(tray, k, product ? PRODUCT_SIZE[product.kind].h : 0.05);
      return t;
    }
    const p = this.products.find((q) => q.id === id);
    if (!p || p.state === 'missed' || p.state === 'placed') return undefined;
    return this.productTarget(p);
  }

  onPickScheduled(item: Target) {
    const p = this.products.find((q) => q.id === item.id);
    if (p) p.state = 'reserved';
  }

  onGrasp(item: Target) {
    const p = this.products.find((q) => q.id === item.id);
    if (p) p.state = 'carried';
  }

  onRelease(item: Target, place: Target) {
    const p = this.products.find((q) => q.id === item.id);
    const trayId = Math.floor((place.id - SLOT_ID_BASE) / 16);
    const k = (place.id - SLOT_ID_BASE) % 16;
    const tray = this.trays.find((t) => t.id === trayId);
    if (!p || !tray) return;
    tray.slots[k] = p.id;
    p.state = 'placed';
    p.trayId = tray.id;
    const s = this.slotPosition(tray, k);
    p.pos = s;
    this.placed++;
  }

  onAbort(item: Target) {
    const p = this.products.find((q) => q.id === item.id);
    if (!p) return;
    if (p.state === 'carried') {
      p.state = 'missed';
      this.missed++;
    } else if (p.state === 'reserved') p.state = 'belt';
  }

  payloadMass(): number {
    return 0.3;
  }

  metrics(): Metric[] {
    const s = this.robots[0]?.stats || { cycleTimes: [], busyTime: 0, totalTime: 1 };
    const minutes = Math.max(this.time, 1) / 60;
    const avgCycle = s.cycleTimes.length ? s.cycleTimes.reduce((a, b) => a + b, 0) / s.cycleTimes.length : 0;
    const total = this.placed + this.missed;
    return [
      { label: 'Picks / min', value: this.placed / minutes, digits: 1 },
      { label: 'Placed', value: this.placed },
      { label: 'Missed', value: this.missed, tone: this.missed > 0 ? 'bad' : 'neutral' },
      {
        label: 'Pick rate',
        value: total ? (100 * this.placed) / total : 100,
        unit: '%',
        digits: 1,
        tone: 'good',
      },
      { label: 'Avg cycle', value: avgCycle, unit: 's', digits: 2 },
      { label: 'Utilisation', value: (100 * s.busyTime) / Math.max(s.totalTime, 1e-9), unit: '%', digits: 0 },
      { label: 'Trays filled', value: this.traysDone },
    ];
  }

  counters() {
    return { placed: this.placed, missed: this.missed, time: this.time };
  }
}