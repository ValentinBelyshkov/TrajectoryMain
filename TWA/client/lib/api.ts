import { AppSettings } from "@shared/api";

const API_BASE_URL = import.meta.env.VITE_API_URL || "";

function getFullUrl(endpoint: string): string {
  let baseUrl = API_BASE_URL;
  let cleanEndpoint = endpoint;
  if (baseUrl.startsWith("/") && cleanEndpoint.startsWith("/api")) {
    cleanEndpoint = cleanEndpoint.replace(/^\/api/, "");
  }
  return `${baseUrl}${cleanEndpoint}`;
}

interface ProjectBackend {
  id: string;
  name: string;
  type: string;
  created_at: string;
  video_filename: string | null;
  frames_path: string | null;
  calibration_status: string;
}

export interface Project {
  id: string;
  name: string;
  type: ProjectType;
  createdAt: Date;
  videoFilename: string | null;
  framesPath: string | null;
  calibrationStatus: string;
}

export type ProjectType = "камера" | "симуляция";

function fromBackendProject(backend: ProjectBackend): Project {
  return {
    id: backend.id,
    name: backend.name,
    type: backend.type as ProjectType,
    createdAt: new Date(backend.created_at),
    videoFilename: backend.video_filename,
    framesPath: backend.frames_path,
    calibrationStatus: backend.calibration_status,
  };
}

function toBackendCreate(project: { name: string; type: ProjectType }): {
  name: string;
  type: string;
} {
  return {
    name: project.name,
    type: project.type,
  };
}

async function request<T>(
  endpoint: string,
  options: RequestInit = {},
): Promise<T> {
  const url = getFullUrl(endpoint);

  const response = await fetch(url, {
    headers: {
      "Content-Type": "application/json",
      ...options.headers,
    },
    ...options,
  });

  if (!response.ok) {
    const error = await response.text();
    throw new Error(error || `Request failed: ${response.status}`);
  }

  return response.json();
}

async function uploadForm<T>(
  endpoint: string,
  field: string,
  file: Blob,
  fileName?: string,
): Promise<T> {
  const formData = new FormData();
  formData.append(field, file, fileName);
  const response = await fetch(getFullUrl(endpoint), {
    method: "POST",
    body: formData,
  });
  if (!response.ok) {
    const error = await response.text();
    throw new Error(error || `Upload failed: ${response.status}`);
  }
  return response.json();
}

export async function getProjects(): Promise<Project[]> {
  const data = await request<ProjectBackend[]>("/api/projects");
  return data.map(fromBackendProject);
}

export async function getProject(projectId: string): Promise<Project> {
  const data = await request<ProjectBackend>(`/api/projects/${projectId}`);
  return fromBackendProject(data);
}

export async function createProject(
  name: string,
  type: ProjectType,
): Promise<Project> {
  const data = await request<ProjectBackend>("/api/projects", {
    method: "POST",
    body: JSON.stringify(toBackendCreate({ name, type })),
  });
  return fromBackendProject(data);
}

export async function updateProject(
  projectId: string,
  name: string,
  type: ProjectType,
): Promise<Project> {
  const data = await request<ProjectBackend>(`/api/projects/${projectId}`, {
    method: "PUT",
    body: JSON.stringify(toBackendCreate({ name, type })),
  });
  return fromBackendProject(data);
}

export async function deleteProject(projectId: string): Promise<void> {
  await request<{ message: string }>(`/api/projects/${projectId}`, {
    method: "DELETE",
  });
}

export async function uploadProjectVideo(
  projectId: string,
  file: File,
): Promise<{ message: string; filename: string }> {
  return uploadForm(`/api/projects/${projectId}/video`, "file", file);
}

// Calibration API

export interface CalibrationPointRequest {
  imageX: number;
  imageY: number;
  lat: number;
  lng: number;
  altitude: number;
}

export async function uploadCalibrationImage(
  projectId: string,
  file: File,
): Promise<{ success: boolean; image_filename: string; image_url: string }> {
  return uploadForm(`/api/projects/${projectId}/upload-image`, "file", file);
}

export async function procframe(
  projectId: string,
): Promise<{ filename: string; url: string }[]> {
  return request<{ filename: string; url: string }[]>(
    `/api/projects/${projectId}/procframe`,
  );
}

export async function saveGCPPoints(
  projectId: string,
  imageFilename: string,
  points: CalibrationPointRequest[],
): Promise<{
  success: boolean;
  gcp_filename: string;
  calibration_status: string;
}> {
  const url = getFullUrl(`/api/projects/${projectId}/save-gcp`);

  const response = await fetch(url, {
    method: "POST",
    headers: {
      "Content-Type": "application/json",
    },
    body: JSON.stringify({
      image_filename: imageFilename,
      points: points,
    }),
  });

  if (!response.ok) {
    const error = await response.text();
    throw new Error(error || `Save failed: ${response.status}`);
  }

  return response.json();
}

export async function saveAllGCPPoints(
  projectId: string,
  images: { image_filename: string; points: CalibrationPointRequest[] }[],
): Promise<{
  success: boolean;
  calibration_status: string;
  points_count: number;
}> {
  const url = getFullUrl(`/api/projects/${projectId}/save-all-gcp`);

  const response = await fetch(url, {
    method: "POST",
    headers: {
      "Content-Type": "application/json",
    },
    body: JSON.stringify({
      images: images,
    }),
  });

  if (!response.ok) {
    const error = await response.text();
    throw new Error(error || `Save failed: ${response.status}`);
  }

  return response.json();
}

export async function getCalibrationStatus(projectId: string): Promise<{
  project_id: string;
  calibrated: boolean;
  calibration_file: string | null;
}> {
  return request<{
    project_id: string;
    calibrated: boolean;
    calibration_file: string | null;
  }>(`/api/projects/${projectId}/calibration/status`);
}

// TerraSLAM control
export interface CommandResponse {
  success: boolean;
  output: string;
  error: string | null;
}

export async function controlTerraSLAMComponent(
  component: string,
  action: string,
  projectId?: string,
  saveFrames?: boolean,
): Promise<CommandResponse> {
  return request<CommandResponse>("/api/control/terraslam/component", {
    method: "POST",
    body: JSON.stringify({
      component, action, project_id: projectId,
      ...(saveFrames !== undefined ? { save_frames: saveFrames } : {}),
    }),
  });
}

export async function getTerraSLAMHealth(): Promise<{
  status: string;
  container_status: string;
}> {
  return request<{
    status: string;
    container_status: string;
  }>("/api/control/terraslam/health");
}

export async function getTerraSLAMStatus(projectId?: string): Promise<{
  system_status: string;
  components: Record<string, string>;
  publisher_mode: string;
  orphaned_processes: Record<string, number>;
  supervisor_output: string;
  error?: string;
}> {
  const query = projectId ? `?project_id=${encodeURIComponent(projectId)}` : "";
  return request<{
    system_status: string;
    components: Record<string, string>;
    publisher_mode: string;
    orphaned_processes: Record<string, number>;
    supervisor_output: string;
    error?: string;
  }>(`/api/control/terraslam/status${query}`);
}

export async function getTerraSLAMLogs(
  component: string,
  lines?: number,
): Promise<{
  component: string;
  stderr_logs: string;
  stdout_logs: string;
  status_indicators: {
    tracking_lost: boolean;
    not_initialized: boolean;
    initializing: boolean;
    valid_data: boolean;
  };
}> {
  const params = new URLSearchParams();
  if (lines) params.append("lines", lines.toString());
  return request(`/api/control/terraslam/logs/${component}?${params}`);
}

export async function getAppSettings(): Promise<AppSettings> {
  return request<AppSettings>("/api/settings");
}

export async function saveAppSettings(
  settings: AppSettings
): Promise<{ success: boolean; settings: AppSettings }> {
  return request("/api/settings", {
    method: "POST",
    body: JSON.stringify(settings),
  });
}

// Auto calibration API

export interface AutoCalibrationRegion {
  lat1: number;
  lng1: number;
  lat2: number;
  lng2: number;
  zoom: number;
}

export interface AutoCalibrationResult {
  success: boolean;
  geotiff_path?: string;
  message?: string;
  error?: string;
  output?: string;
}

export interface AutoMatchResult {
  success: boolean;
  message: string;
  details?: string;
}

export async function downloadGeotiff(
  projectId: string,
  region: AutoCalibrationRegion
): Promise<AutoCalibrationResult> {
  return request<AutoCalibrationResult>(
    `/api/projects/${projectId}/auto/download-geotiff`,
    {
      method: "POST",
      body: JSON.stringify(region),
    }
  );
}

export async function getGeotiffStatus(
  projectId: string
): Promise<{ exists: boolean; path: string | null }> {
  return request<{ exists: boolean; path: string | null }>(
    `/api/projects/${projectId}/auto/geotiff-status`
  );
}

export async function getAutoCalibrationFrames(
  projectId: string
): Promise<{ filename: string; url: string }[]> {
  return request<{ filename: string; url: string }[]>(
    `/api/projects/${projectId}/auto/frames`
  );
}

export async function matchImageToGeotiff(
  projectId: string,
  imageFilename: string
): Promise<AutoMatchResult> {
  return request<AutoMatchResult>(
    `/api/projects/${projectId}/auto/match-image`,
    {
      method: "POST",
      body: JSON.stringify({ image_filename: imageFilename }),
    }
  );
}

export async function getProjectFrames(
  projectId: string
): Promise<{ filename: string; url: string }[]> {
  const data = await request<any>(`/api/projects/${projectId}/frames`);
  // The backend may still serve the legacy object shape
  // ({ has_frames, frame_count }) until it is restarted; guard against that
  // so the trimming step degrades gracefully instead of crashing.
  if (!Array.isArray(data)) return [];
  return data as { filename: string; url: string }[];
}

export interface ProjectFramesCheck {
  has_frames: boolean;
  frame_count: number;
}

export async function checkProjectHasFrames(
  projectId: string
): Promise<ProjectFramesCheck> {
  return request<ProjectFramesCheck>(
    `/api/projects/${projectId}/has-frames`
  );
}

export async function trimProjectFrames(
  projectId: string,
  keep: string[],
): Promise<{ deleted: number; remaining: number }> {
  return request<{ deleted: number; remaining: number }>(
    `/api/projects/${projectId}/frames/trim`,
    {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ keep }),
    }
  );
}

// Calibration session API (TerraSLAM_relay)

export interface CalibrationSession {
  id: string;
  created_at: number;
  status: string;
  video_path: string;
  trimmed_video_path?: string;
  trim_segments: Array<{ start: number; end: number }>;
  project_id: string;
  frames_dir?: string;
  procframe_dir: string;
  frame_pose_data: Array<{ frame: string; pose_file?: string; pose?: { x: number; y: number; z: number }; timestamp: number }>;
  correlation_points: any[];
  transform?: any;
  calib_gpc_path?: string;
}

export async function startCalibrationSession(
  projectId?: string,
  clearExistingFrames: boolean = true,
): Promise<{ success: boolean; session: CalibrationSession }> {
  return request<{ success: boolean; session: CalibrationSession }>("/api/v1/calibration/video/start", {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify({
      ...(projectId ? { project_id: projectId } : {}),
      clear_existing_frames: clearExistingFrames,
    }),
  });
}

export async function startCalibrationSessionFromProject(projectId: string): Promise<{ success: boolean; session: CalibrationSession }> {
  return request<{ success: boolean; session: CalibrationSession }>(`/api/v1/calibration/video/start-from-project/${projectId}`, {
    method: "POST",
  });
}

export async function uploadCalibrationChunk(
  sessionId: string,
  chunk: Blob
): Promise<{ success: boolean; bytes_written: number }> {
  return uploadForm(
    `/api/v1/calibration/video/${sessionId}/chunk`,
    "chunk",
    chunk,
    "chunk.webm",
  );
}

export async function uploadCalibrationVideo(
  sessionId: string,
  file: File
): Promise<{ success: boolean; bytes_written: number; session: CalibrationSession }> {
  return uploadForm(`/api/v1/calibration/video/${sessionId}/upload`, "file", file);
}

export async function stopCalibrationSession(sessionId: string): Promise<{ success: boolean; session: CalibrationSession }> {
  return request<{ success: boolean; session: CalibrationSession }>(`/api/v1/calibration/video/${sessionId}/stop`, {
    method: "POST",
  });
}

export async function getCalibrationSessions(): Promise<{ success: boolean; sessions: any[] }> {
  return request<{ success: boolean; sessions: any[] }>("/api/v1/calibration/sessions");
}

export async function getCalibrationSession(sessionId: string): Promise<{ success: boolean; session: CalibrationSession }> {
  return request<{ success: boolean; session: CalibrationSession }>(`/api/v1/calibration/session/${sessionId}`);
}

export interface CalibrationRecordingStatus {
  recording: boolean;
  status: string | null;
  session_id: string | null;
  elapsed: number;
  publisher_running: boolean;
  server_time: number;
}

export async function getCalibrationRecordingStatus(projectId: string): Promise<CalibrationRecordingStatus> {
  return request<CalibrationRecordingStatus>(`/api/v1/calibration/recording-status/${projectId}`);
}

export async function trimCalibrationVideo(
  sessionId: string,
  segments: Array<{ start: number; end: number }>
): Promise<{ success: boolean; session: CalibrationSession }> {
  return request<{ success: boolean; session: CalibrationSession }>(`/api/v1/calibration/video/${sessionId}/trim`, {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify({ segments }),
  });
}

export async function processCalibrationSession(
  sessionId: string,
  projectId: string
): Promise<{ success: boolean; session: CalibrationSession; frames_extracted: number; poses_saved: number }> {
  return request<{ success: boolean; session: CalibrationSession; frames_extracted: number; poses_saved: number }>("/api/v1/calibration/process", {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify({ session_id: sessionId, project_id: projectId }),
  });
}

export interface CalibrationProgress {
  found: boolean;
  session_status?: string;
  started_at?: number;
  elapsed?: number;
  step?: string;
  step_label?: string;
  frames_total?: number;
  frames_done?: number;
  slam_running?: boolean;
  slam_crashed?: boolean;
  error?: string;
  poses_saved?: number;
  procframe_dir?: string;
}

export async function getCalibrationProgress(sessionId: string): Promise<CalibrationProgress> {
  return request<CalibrationProgress>(`/api/v1/calibration/session/${sessionId}/progress`);
}

export async function correlateCalibrationPoints(
  sessionId: string,
  points: any[]
): Promise<{ success: boolean; transform: any; session: CalibrationSession }> {
  return request<{ success: boolean; transform: any; session: CalibrationSession }>(`/api/v1/calibration/session/${sessionId}/correlate`, {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify({ points }),
  });
}

export async function finalizeCalibrationSession(sessionId: string): Promise<{ success: boolean; system_calib: string; project_calib: string; content: string }> {
  return request<{ success: boolean; system_calib: string; project_calib: string; content: string }>(`/api/v1/calibration/session/${sessionId}/finalize`, {
    method: "POST",
  });
}
