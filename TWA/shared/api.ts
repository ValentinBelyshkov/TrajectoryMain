/**
 * Shared code between client and server
 * Useful to share types between client and server
 * and/or small pure JS functions that can be used on both client and server
 */

/**
 * Example response type for /api/demo
 */
export interface DemoResponse {
  message: string;
}

export type ProjectType = "камера" | "симуляция";

export interface Project {
  id: string;
  name: string;
  type: ProjectType;
  createdAt: string;
}

export interface CreateProjectRequest {
  name: string;
  type: ProjectType;
}

export interface CreateProjectResponse {
  success: boolean;
  project?: Project;
  error?: string;
}

export interface ProjectsResponse {
  projects: Project[];
}

// Calibration types
export interface CalibrationPoint {
  id: string;
  imageX: number;
  imageY: number;
  lat: number;
  lng: number;
  altitude: number;
}

export interface GCPRequest {
  projectId: string;
  imageFilename: string;
  points: CalibrationPoint[];
}

export interface CalibrationImageResponse {
  success: boolean;
  imageFilename: string;
  imageUrl: string;
}

export interface GCPSaveResponse {
  success: boolean;
  gcpFilename: string;
  calibrationStatus: string;
}

// Settings types
export type ProtocolType = "Mavlink" | "MSP" | "Ublox" | "Custom";
export type TransmitterType = "UART" | "I2C" | "USB";
export type SessionType = "Mono" | "Stereo" | "Mono+IMU" | "Stereo+IMU";

export interface ORBExtractorSettings {
  nFeatures: number;
  scaleFactor: number;
  nLevels: number;
  iniThFAST: number;
  minThFAST: number;
}

export interface ProtocolConfig {
  id: string;
  name: string;
  baudRate?: number;
  port?: string;
  // Other protocol specific fields
}

export interface AppSettings {
  protocol: ProtocolType;
  transmitter: TransmitterType;
  session: SessionType;
  orbExtractor: ORBExtractorSettings;
  protocolConfigs: Record<ProtocolType, ProtocolConfig>;
}

export interface SettingsResponse {
  success: boolean;
  settings: AppSettings;
}

export interface SaveSettingsRequest {
  settings: AppSettings;
}
