import { create } from "zustand";

export interface Project {
  id: string;
  name: string;
  type: "камера" | "симуляция";
  createdAt: Date;
  videoFile?: File;
  videoFilename?: string;
  calibrationStatus: "not_calibrated" | "pending" | "calibrated";
}

export interface TelemetryData {
  height: number;
  vx: number;
  vy: number;
  vz: number;
  battery: number;
  gpsStatus: "lock" | "no_fix" | "rtk";
  mode: "manual" | "auto" | "simulation" | "hover";
  status: "idle" | "recording" | "active" | "error";
  lat: number;
  lng: number;
  timestamp: number;
}

export interface DronePosition {
  lat: number;
  lng: number;
  altitude: number;
}

interface AppState {
  // Projects
  projects: Project[];
  currentProjectId: string | null;
  addProject: (project: Project) => void;
  removeProject: (id: string) => void;
  updateProject: (id: string, updates: Partial<Project>) => void;
  setCurrentProject: (id: string | null) => void;

  // Telemetry
  telemetry: TelemetryData | null;
  setTelemetry: (data: TelemetryData) => void;
  dronePosition: DronePosition;
  setDronePosition: (position: DronePosition) => void;
  dronePath: Array<{ lat: number; lng: number }>;
  addDronePathPoint: (point: { lat: number; lng: number }) => void;
  clearDronePath: () => void;

  // UI State
  isRecording: boolean;
  setIsRecording: (recording: boolean) => void;
  calibrationStatus: "not_calibrated" | "pending" | "calibrated";
  setCalibrationStatus: (
    status: "not_calibrated" | "pending" | "calibrated",
  ) => void;

  // WebSocket
  wsConnected: boolean;
  setWsConnected: (connected: boolean) => void;
}

const defaultTelemetry: TelemetryData = {
  height: 0,
  vx: 0,
  vy: 0,
  vz: 0,
  battery: 100,
  gpsStatus: "no_fix",
  mode: "manual",
  status: "idle",
  lat: 55.7558,
  lng: 37.6173,
  timestamp: Date.now(),
};

export const useAppStore = create<AppState>((set) => ({
  // Projects
  projects: [],
  currentProjectId: null,
  addProject: (project) =>
    set((state) => ({ projects: [project, ...state.projects] })),
  removeProject: (id) =>
    set((state) => ({
      projects: state.projects.filter((p) => p.id !== id),
      currentProjectId:
        state.currentProjectId === id ? null : state.currentProjectId,
    })),
  updateProject: (id, updates) =>
    set((state) => ({
      projects: state.projects.map((p) =>
        p.id === id ? { ...p, ...updates } : p,
      ),
    })),
  setCurrentProject: (id) => set({ currentProjectId: id }),

  // Telemetry
  telemetry: null,
  setTelemetry: (data) => set({ telemetry: data }),
  dronePosition: { lat: 55.7558, lng: 37.6173, altitude: 0 },
  setDronePosition: (position) => set({ dronePosition: position }),
  dronePath: [],
  addDronePathPoint: (point) =>
    set((state) => ({
      dronePath: [...state.dronePath, point],
    })),
  clearDronePath: () => set({ dronePath: [] }),

  // UI State
  isRecording: false,
  setIsRecording: (recording) => set({ isRecording: recording }),
  calibrationStatus: "not_calibrated",
  setCalibrationStatus: (status) => set({ calibrationStatus: status }),

  // WebSocket
  wsConnected: false,
  setWsConnected: (connected) => set({ wsConnected: connected }),
}));

export const selectCurrentProject = (state: AppState) =>
  state.projects.find((p) => p.id === state.currentProjectId);

export const selectTelemetry = (state: AppState) => state.telemetry;
