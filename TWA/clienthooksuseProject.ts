import { useState, useRef, useEffect, useCallback } from "react";
import { useQuery } from "@tanstack/react-query";
import {
  getProject,
  uploadCalibrationImage,
  saveGCPPoints,
  controlTerraSLAMComponent,
  getTerraSLAMStatus,
  type Project,
  procframe,
  downloadGeotiff,
  matchImageToGeotiff,
  getAutoCalibrationFrames,
  type AutoCalibrationRegion,
  startCalibrationSession,
  uploadCalibrationChunk,
  stopCalibrationSession,
  getCalibrationSession,
  trimCalibrationVideo,
  processCalibrationSession,
  correlateCalibrationPoints,
  finalizeCalibrationSession,
} from "@/lib/api";
import type { CalibrationPoint } from "@/components/CalibrationPointSelector";

export type CalibrationStep = 
  | "idle" 
  | "type-selection"    // NEW: Выбор типа калибровки
  | "instructions" 
  | "test-run" 
  | "frame-selection" 
  | "upload" 
  | "pairing" 
  | "complete"
  | "auto-region"        // NEW: Выбор региона на карте для auto калибровки
  | "auto-downloading"  // NEW: Загрузка GeoTIFF
  | "auto-image-select" // NEW: Выбор изображения для сопоставления
  | "recording"
  | "trimming"
  | "processing"
  | "correlating"
  | "finalizing";

export interface TelemetryData {
  height: number;
  speed: number;
  battery: number;
  status: "idle" | "recording" | "active";
}

export interface DronePosition {
  lat: number;
  lng: number;
}

export interface DronePath extends Array<DronePosition> {}

// NEW: GPS Status interface
export interface GPSStatus {
  hasSignal: boolean;
  lastUpdate: number | null;
  lat: number | null;
  lon: number | null;
  alt: number | null;
}

export function useProject(projectId: string | undefined) {
  const [isRecording, setIsRecording] = useState(false);
  const [dronePosition, setDronePosition] = useState<DronePosition>({
    lat: 55.7558,
    lng: 37.6173,
  });
  const [dronePath, setDronePath] = useState<DronePath>([]);
  const [telemetry, setTelemetry] = useState<TelemetryData>({
    height: 0,
    speed: 0,
    battery: 100,
    status: "idle",
  });
  const [calibrationStep, setCalibrationStep] = useState<CalibrationStep>("idle");
  const [uploadedImage, setUploadedImage] = useState<{
    filename: string;
    url: string;
  } | null>(null);
  const [selectedFrames, setSelectedFrames] = useState<{
    filename: string;
    url: string;
  }[]>([]);
  const [uploadError, setUploadError] = useState<string | null>(null);
  const [isUploading, setIsUploading] = useState(false);
  const [hasVideoStream, setHasVideoStream] = useState(false);
  
  const [gpsStatus, setGpsStatus] = useState<GPSStatus>({
    hasSignal: false,
    lastUpdate: null,
    lat: null,
    lon: null,
    alt: null,
  });

  // NEW: Calibration session state
  const [calibrationSessionId, setCalibrationSessionId] = useState<string | null>(null);
  const [calibrationSession, setCalibrationSession] = useState<any>(null);
  const [recordingStatus, setRecordingStatus] = useState<"idle" | "recording" | "stopped">("idle");
  const [recordingDuration, setRecordingDuration] = useState(0);
  const [trimSegments, setTrimSegments] = useState<Array<{ start: number; end: number }>>([]);
  const [processingProgress, setProcessingProgress] = useState<string>("");
  const [correlationPoints, setCorrelationPoints] = useState<any[]>([]);
  const [transform, setTransform] = useState<any>(null);

  // NEW: Auto calibration state
  const [autoCalibrationRegion, setAutoCalibrationRegion] = useState<AutoCalibrationRegion | null>(null);
  const [autoCalibrationFrames, setAutoCalibrationFrames] = useState<{ filename: string; url: string }[]>([]);
  const [autoCalibrationError, setAutoCalibrationError] = useState<string | null>(null);
  const [autoCalibrationProgress, setAutoCalibrationProgress] = useState<"idle" | "downloading" | "matching" | "success" | "error">("idle");
  const [autoCalibrationMessage, setAutoCalibrationMessage] = useState<string | null>(null);

  // System status from TerraSLAM
  const [systemStatus, setSystemStatus] = useState<{
    status: "working" | "warning" | "not_working" | "error";
    publisher_mode: string;
    components: Record<string, string>;
  } | null>(null);

  const videoCanvasRef = useRef<HTMLCanvasElement | null>(null);
  const wsRef = useRef<WebSocket | null>(null);
  const gpsWsRef = useRef<WebSocket | null>(null);
  const gpsTimeoutRef = useRef<ReturnType<typeof setTimeout> | null>(null);
  const recordingIntervalRef = useRef<ReturnType<typeof setInterval> | null>(null);
  const mediaRecorderRef = useRef<MediaRecorder | null>(null);
  const recordedChunksRef = useRef<Blob[]>([]);

  const {
    data: project,
    isLoading,
    error,
    refetch,
  } = useQuery({
    queryKey: ["project", projectId],
    queryFn: () => getProject(projectId!),
    enabled: !!projectId,
  });

  const isCalibrated = project?.calibrationStatus === "calibrated";
  const showCalibration = project ? !isCalibrated : true;

  useEffect(() => {
    if (project) {
      if (isCalibrated) {
        setCalibrationStep("complete");
      } else if (calibrationStep === "complete") {
        // If it was complete but now project says not calibrated, go back to idle
        setCalibrationStep("idle");
      }
    }
  }, [project, isCalibrated]);

  // Poll TerraSLAM system status
  useEffect(() => {
    if (!projectId) return;

    const fetchStatus = async () => {
      try {
        const status = await getTerraSLAMStatus();
        setSystemStatus({
          status: status.system_status,
          publisher_mode: status.publisher_mode,
          components: status.components,
        });
      } catch (err) {
        console.error("Failed to fetch TerraSLAM status:", err);
        setSystemStatus({
          status: "error",
          publisher_mode: "unknown",
          components: {},
        });
      }
    };

    // Initial fetch
    fetchStatus();

    // Poll every 3 seconds
    const interval = setInterval(fetchStatus, 3000);
    return () => clearInterval(interval);
  }, [projectId]);

  // Video WebSocket (existing)
  useEffect(() => {
    if (!projectId) return;

    const wsUrl = `${import.meta.env.VITE_WS_URL || "ws://localhost:9000"}/api/video/ws/${projectId}`;
    const ws = new WebSocket(wsUrl);
    wsRef.current = ws;

    ws.onopen = () => console.log("✅ Video WebSocket connected");
    
    ws.onmessage = (event) => {
      try {
        const message = JSON.parse(event.data);
        if (message.type === "frame" && message.data) {
        // Inside ws.onmessage, after parsing:
          setHasVideoStream(true);
          const canvas = videoCanvasRef.current;
          if (!canvas) return;

          const ctx = canvas.getContext("2d");
          if (!ctx) return;

          // Decode and display compressed image (JPEG/PNG)
          const img = new Image();
          img.onload = () => {
            canvas.width = img.width;
            canvas.height = img.height;
            ctx.drawImage(img, 0, 0);
          };
          img.src = `data:image/jpeg;base64,${message.data}`;
        }
      } catch (e) {
        console.error("Video stream error:", e);
      }
    };

    ws.onerror = (e) => console.error("Video WebSocket error:", e);
    ws.onclose = () => console.log("Video WebSocket closed");

    return () => {
      ws.close();
      wsRef.current = null;
    };
  }, [projectId]);

  // NEW: GPS WebSocket with 1-second timeout
  useEffect(() => {
    if (!projectId) return;

    const gpsWsEnabled = import.meta.env.VITE_GPS_WS_ENABLED !== "false";

    if (!gpsWsEnabled) {
      console.log("GPS WebSocket disabled by VITE_GPS_WS_ENABLED");
      return;
    }

    const connectGPS = () => {
      const rosbridgeHost = import.meta.env.VITE_ROSBRIDGE_HOST;
      const rosbridgePort = import.meta.env.VITE_ROSBRIDGE_PORT;

      if (!rosbridgeHost && !rosbridgePort) {
        console.log("GPS WebSocket skipped: rosbridge not configured");
        return;
      }

      const gpsUrl = `ws://${rosbridgeHost || "localhost"}:${rosbridgePort || "9091"}`;

      console.log("Connecting to GPS WebSocket:", gpsUrl);
      const ws = new WebSocket(gpsUrl);
      gpsWsRef.current = ws;

      ws.onopen = () => {
        console.log("✅ GPS WebSocket connected");
        const subscribeMsg = {
          op: "subscribe",
          topic: "/camera/gps",
          type: "sensor_msgs/msg/NavSatFix",
          queue_length: 1,
        };
        ws.send(JSON.stringify(subscribeMsg));
      };

      ws.onmessage = (event) => {
        try {
          const msg = JSON.parse(event.data);
          if (msg.topic === "/camera/gps" && msg.msg) {
            const { latitude, longitude, altitude } = msg.msg;

            if (gpsTimeoutRef.current) {
              clearTimeout(gpsTimeoutRef.current);
            }

            setGpsStatus({
              hasSignal: true,
              lastUpdate: Date.now(),
              lat: latitude,
              lon: longitude,
              alt: altitude,
            });

            setDronePosition({ lat: latitude, lng: longitude });

            if (isRecording) {
              setDronePath((prev) => [...prev, { lat: latitude, lng: longitude }]);
            }

            gpsTimeoutRef.current = setTimeout(() => {
              setGpsStatus((prev) => ({ ...prev, hasSignal: false }));
            }, 1000);
          }
        } catch (e) {
          console.error("GPS message error:", e);
        }
      };

      ws.onerror = (e) => {
        console.error("GPS WebSocket error:", e);
        setTimeout(connectGPS, 3000);
      };

      ws.onclose = () => {
        console.log("GPS WebSocket closed, retrying...");
        setGpsStatus((prev) => ({ ...prev, hasSignal: false }));
        setTimeout(connectGPS, 3000);
      };
    };

    connectGPS();

    return () => {
      if (gpsTimeoutRef.current) clearTimeout(gpsTimeoutRef.current);
      if (gpsWsRef.current) {
        gpsWsRef.current.close();
        gpsWsRef.current = null;
      }
    };
  }, [projectId, isRecording]);

  const startRecording = useCallback(async () => {
    try {
      // Restart all components instead of just starting
      await controlTerraSLAMComponent("all", "restart", projectId);
      
      // Reconnect video WebSocket if it was closed
      if (wsRef.current && wsRef.current.readyState !== WebSocket.OPEN) {
        wsRef.current = null; // Trigger reconnection via useEffect
      }
    } catch (err) {
      console.error("Failed to restart TerraSLAM:", err);
    }

    setIsRecording(true);
    setTelemetry((prev) => ({ ...prev, status: "recording" }));
    
    // Start path from current GPS position if available
    if (gpsStatus.lat && gpsStatus.lon) {
      setDronePath([{ lat: gpsStatus.lat, lng: gpsStatus.lon }]);
    }
  }, [gpsStatus, projectId]);

  const stopRecording = useCallback(async () => {
    if (recordingIntervalRef.current) {
      clearInterval(recordingIntervalRef.current);
      recordingIntervalRef.current = null;
    }

    try {
      // Stop TerraSLAM components
      await controlTerraSLAMComponent("all", "stop", projectId);
      
      // Close video WebSocket to stop receiving frames
      if (wsRef.current && wsRef.current.readyState === WebSocket.OPEN) {
        wsRef.current.close();
      }
    } catch (err) {
      console.error("Failed to stop TerraSLAM:", err);
    }

    setIsRecording(false);
    setTelemetry((prev) => ({ ...prev, status: "idle" }));
  }, [projectId]);

  const handleCalibrate = useCallback(() => {
    setCalibrationStep("type-selection");
  }, []);

  const handleCalibrationTypeSelect = useCallback((type: "manual" | "auto") => {
    if (type === "auto") {
      setCalibrationStep("auto-region");
    } else {
      setCalibrationStep("recording");
    }
  }, []);

  const handleInstructionsNext = useCallback(() => {
    setCalibrationStep("test-run");
  }, []);

  const handleTestRunSuccess = useCallback(() => {
    setCalibrationStep("frame-selection");
  }, []);

  const handleTestRunBack = useCallback(() => {
    setCalibrationStep("type-selection");
  }, []);

  const handleFrameSelectionBack = useCallback(() => {
    setCalibrationStep("type-selection");
  }, []);

  const handleFramesSelected = useCallback((frames: { filename: string; url: string }[]) => {
    setSelectedFrames(frames);
    setCalibrationStep("pairing");
  }, []);

  // Auto calibration handlers
  const handleAutoRegionConfirm = useCallback(async (region: AutoCalibrationRegion) => {
    if (!projectId) return;
    
    setAutoCalibrationRegion(region);
    setCalibrationStep("auto-downloading");
    setAutoCalibrationProgress("downloading");
    setAutoCalibrationError(null);
    setAutoCalibrationMessage("Загрузка карты...");
    
    try {
      const result = await downloadGeotiff(projectId, region);
      
      if (result.success) {
        setAutoCalibrationMessage("Карта загружена. Загрузка списка кадров...");
        setCalibrationStep("auto-image-select");
        setAutoCalibrationProgress("idle");
        
        // Fetch available frames
        const frames = await getAutoCalibrationFrames(projectId);
        setAutoCalibrationFrames(frames);
      } else {
        setAutoCalibrationError(result.error || result.message || "Ошибка загрузки карты");
        setAutoCalibrationProgress("error");
        setCalibrationStep("auto-region");
      }
    } catch (err) {
      setAutoCalibrationError(err instanceof Error ? err.message : "Ошибка загрузки карты");
      setAutoCalibrationProgress("error");
      setCalibrationStep("auto-region");
    }
  }, [projectId]);

  const handleAutoImageSelect = useCallback(async (imageFilename: string) => {
    if (!projectId) return;
    
    setAutoCalibrationProgress("matching");
    setAutoCalibrationMessage("Сопоставление изображения с картой...");
    setAutoCalibrationError(null);
    
    try {
      const result = await matchImageToGeotiff(projectId, imageFilename);
      
      if (result.success) {
        setAutoCalibrationProgress("success");
        setAutoCalibrationMessage("Калибровка успешно пройдена!");
        
        // Refetch project data to update calibration status
        await refetch();
        setCalibrationStep("complete");
      } else {
        setAutoCalibrationError(result.message || "Ошибка калибровки. Попробуйте другое фото или параметры снимка");
        setAutoCalibrationProgress("error");
      }
    } catch (err) {
      setAutoCalibrationError(err instanceof Error ? err.message : "Ошибка калибровки");
      setAutoCalibrationProgress("error");
    }
  }, [projectId, refetch]);

  const handleAutoCalibrationBack = useCallback(() => {
    setCalibrationStep("type-selection");
    setAutoCalibrationRegion(null);
    setAutoCalibrationFrames([]);
    setAutoCalibrationError(null);
    setAutoCalibrationProgress("idle");
    setAutoCalibrationMessage(null);
  }, []);

  const startNewCalibrationSession = useCallback(async () => {
    const data = await startCalibrationSession();
    if (!data.success) throw new Error("Failed to start session");
    setCalibrationSessionId(data.session.id);
    setCalibrationSession(data.session);
    localStorage.setItem("calib_session_id", data.session.id);
    return data.session;
  }, []);

  const resumeCalibrationSession = useCallback(async (sessionId: string) => {
    const data = await getCalibrationSession(sessionId);
    if (!data.success) throw new Error("Failed to load session");
    setCalibrationSessionId(sessionId);
    setCalibrationSession(data.session);
    return data.session;
  }, []);

  const handleStartRecording = useCallback(async () => {
    try {
      let session = calibrationSession;
      if (!session) {
        const stored = localStorage.getItem("calib_session_id");
        if (stored) {
          try {
            session = await resumeCalibrationSession(stored);
          } catch {
            session = await startNewCalibrationSession();
          }
        } else {
          session = await startNewCalibrationSession();
        }
      }
      setRecordingStatus("recording");
      setCalibrationStep("recording");
      setRecordingDuration(0);
      recordedChunksRef.current = [];
    } catch (err) {
      console.error("Failed to start recording session:", err);
    }
  }, [calibrationSession, startNewCalibrationSession, resumeCalibrationSession]);

  const handleStopRecording = useCallback(async () => {
    setRecordingStatus("stopped");
    setCalibrationStep("trimming");
  }, []);

  const handleApplyTrim = useCallback(async (segments: Array<{ start: number; end: number }>) => {
    if (!calibrationSessionId) return;
    setTrimSegments(segments);
    const data = await trimCalibrationVideo(calibrationSessionId, segments);
    if (!data.success) throw new Error("Trim failed");
    setCalibrationSession(data.session);
    setCalibrationStep("processing");
  }, [calibrationSessionId]);

  const handleRunProcessing = useCallback(async () => {
    if (!calibrationSessionId || !projectId) return;
    setProcessingProgress("Extracting frames...");
    try {
      const data = await processCalibrationSession(calibrationSessionId, projectId);
      if (!data.success) throw new Error("Processing failed");
      setProcessingProgress("");
      setCalibrationSession(data.session);
      setCalibrationStep("correlating");
    } catch (err) {
      setProcessingProgress("");
      console.error("Processing failed:", err);
    }
  }, [calibrationSessionId, projectId]);

  const handleComputeCorrelation = useCallback(async (points: any[]) => {
    if (!calibrationSessionId) return;
    setCorrelationPoints(points);
    const data = await correlateCalibrationPoints(calibrationSessionId, points);
    if (!data.success) throw new Error("Correlation failed");
    setTransform(data.transform);
    setCalibrationSession(data.session);
    setCalibrationStep("finalizing");
  }, [calibrationSessionId]);

  const handleFinalizeCalibration = useCallback(async () => {
    if (!calibrationSessionId) return;
    const data = await finalizeCalibrationSession(calibrationSessionId);
    if (!data.success) throw new Error("Finalize failed");
    setCalibrationStep("complete");
    localStorage.removeItem("calib_session_id");
    await refetch();
  }, [calibrationSessionId, refetch]);

  const handleImageUpload = useCallback(
    async (file: File) => {
      if (!file || !projectId) return;

      setIsUploading(true);
      setUploadError(null);

      try {
        const result = await uploadCalibrationImage(projectId, file);
        setUploadedImage({
          filename: result.image_filename,
          url: result.image_url,
        });
        setCalibrationStep("pairing");
      } catch (err) {
        setUploadError(
          err instanceof Error ? err.message : "Ошибка загрузки изображения"
        );
      } finally {
        setIsUploading(false);
      }
    },
    [projectId]
  );

  const handleCalibrationComplete = useCallback(
    async () => {
      if (!projectId) return;

      try {
        await refetch();
        setCalibrationStep("complete");
        setTelemetry((prev) => ({ ...prev, status: "active" }));
      } catch (err) {
        alert(err instanceof Error ? err.message : "Ошибка обновления данных");
      }
    },
    [projectId, refetch]
  );

  const handleCalibrationCancel = useCallback(() => {
    setCalibrationStep(project?.calibrationStatus === "calibrated" ? "complete" : "idle");
    setUploadedImage(null);
    setSelectedFrames([]);
  }, [project?.calibrationStatus]);

  const clearUploadError = useCallback(() => {
    setUploadError(null);
  }, []);

  // Expose setter for step transitions that don't have a dedicated handler
  const setStep = useCallback((step: CalibrationStep) => {
    setCalibrationStep(step);
  }, []);

  return {
    project,
    isLoading,
    error,
    isRecording,
    dronePosition,
    dronePath,
    telemetry,
    showCalibration,
    calibrationStep,
    setCalibrationStep: setStep,
    uploadedImage,
    selectedFrames,
    uploadError,
    isUploading,
    hasVideoStream,
    videoCanvasRef,
    startRecording,
    stopRecording,
    handleCalibrate,
    handleCalibrationTypeSelect,
    handleInstructionsNext,
    handleTestRunSuccess,
    handleTestRunBack,
    handleFrameSelectionBack,
    handleFramesSelected,
    handleImageUpload,
    handleCalibrationComplete,
    handleCalibrationCancel,
    clearUploadError,
    refetch,
    gpsStatus, // NEW: Export GPS status
    systemStatus, // NEW: Export system status from TerraSLAM
    // Auto calibration exports
    autoCalibrationRegion,
    autoCalibrationFrames,
    autoCalibrationError,
    autoCalibrationProgress,
    autoCalibrationMessage,
    handleAutoRegionConfirm,
    handleAutoImageSelect,
    handleAutoCalibrationBack,
    // NEW calibration workflow exports
    calibrationSessionId,
    setCalibrationSessionId,
    calibrationSession,
    recordingStatus,
    setRecordingStatus,
    recordingDuration,
    setRecordingDuration,
    trimSegments,
    processingProgress,
    correlationPoints,
    transform,
    handleStartRecording,
    handleStopRecording,
    handleApplyTrim,
    handleRunProcessing,
    handleComputeCorrelation,
    handleFinalizeCalibration,
    startNewCalibrationSession,
    resumeCalibrationSession,
  };
}
