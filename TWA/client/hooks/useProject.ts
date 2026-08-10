import { useState, useRef, useCallback, useEffect } from "react";
import { useQuery } from "@tanstack/react-query";
import { useWebSocket } from "@/hooks/useWebSocket";
import { useTerraSLAMStatus, isSlamRunning } from "@/hooks/useTerraSLAMStatus";
import { useCalibrationSession } from "@/hooks/useCalibrationSession";
import { getProject, controlTerraSLAMComponent, type Project } from "@/lib/api";

export type CalibrationStep =
  | "idle"
  | "type-selection"
  | "instructions"
  | "test-run"
  | "frame-selection"
  | "upload"
  | "pairing"
  | "complete"
  | "auto-region"
  | "auto-downloading"
  | "auto-image-select"
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

export interface GPSStatus {
  hasSignal: boolean;
  lastUpdate: number | null;
  lat: number | null;
  lon: number | null;
  alt: number | null;
}

export function useProject(projectId: string | undefined) {
  // Optimistic override applied while a start/stop request is in flight.
  // Once the backend status catches up, it becomes the source of truth again.
  const [pendingRecording, setPendingRecording] = useState<boolean | null>(null);
  const [isBusy, setIsBusy] = useState(false);
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
  const [hasVideoStream, setHasVideoStream] = useState(false);
  const [saveFrames, setSaveFrames] = useState(false);

  const [gpsStatus, setGpsStatus] = useState<GPSStatus>({
    hasSignal: false,
    lastUpdate: null,
    lat: null,
    lon: null,
    alt: null,
  });

  const videoCanvasRef = useRef<HTMLCanvasElement | null>(null);
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

  const systemStatus = useTerraSLAMStatus(projectId);

  // SLAM is the source of truth for the Start/Stop button. While a start/stop
  // request is in flight we optimistically show the requested state, then fall
  // back to the real backend state as soon as it agrees.
  const slamRunning = isSlamRunning(systemStatus);
  const isRecording = pendingRecording ?? slamRunning;

  // Clear the optimistic override once the backend reports the expected state.
  useEffect(() => {
    if (pendingRecording !== null && pendingRecording === slamRunning) {
      setPendingRecording(null);
    }
  }, [pendingRecording, slamRunning]);

  // Keep telemetry in sync with the real SLAM state.
  useEffect(() => {
    setTelemetry((prev) => {
      const status = isRecording ? "recording" : "idle";
      return prev.status === status ? prev : { ...prev, status };
    });
  }, [isRecording]);

  // Video WebSocket — draws incoming frames onto the canvas
  const wsRef = useWebSocket(
    projectId
      ? `${import.meta.env.VITE_WS_URL || "ws://localhost:9000"}/api/video/ws/${projectId}`
      : null,
    {
      enabled: !!projectId,
      reconnect: true,
      reconnectDelay: 2000,
      onMessage: (event) => {
        try {
          const message = JSON.parse(event.data);
          if (message.type === "frame" && message.data) {
            setHasVideoStream(true);
            const canvas = videoCanvasRef.current;
            if (!canvas) return;

            const ctx = canvas.getContext("2d");
            if (!ctx) return;

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
      },
    },
  );

  // GPS WebSocket with 1-second timeout (auto-reconnect)
  const gpsWsEnabled = import.meta.env.VITE_GPS_WS_ENABLED !== "false";
  const rosbridgeHost = import.meta.env.VITE_ROSBRIDGE_HOST;
  const rosbridgePort = import.meta.env.VITE_ROSBRIDGE_PORT;
  const gpsEnabled = gpsWsEnabled && (!!rosbridgeHost || !!rosbridgePort);
  useWebSocket(
    gpsEnabled ? `ws://${rosbridgeHost || "localhost"}:${rosbridgePort || "9091"}` : null,
    {
      enabled: gpsEnabled,
      reconnect: true,
      reconnectDelay: 3000,
      onOpen: (ws) => {
        console.log("✅ GPS WebSocket connected");
        ws.send(
          JSON.stringify({
            op: "subscribe",
            topic: "/camera/gps",
            type: "sensor_msgs/msg/NavSatFix",
            queue_length: 1,
          }),
        );
      },
      onMessage: (event) => {
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
      },
    },
  );

  const startRecording = useCallback(async () => {
    // Guard: never launch a second SLAM session if one is already running.
    if (isBusy || isRecording) return;

    setIsBusy(true);
    setPendingRecording(true);
    try {
      await controlTerraSLAMComponent("all", "restart", projectId, saveFrames);

      if (wsRef.current && wsRef.current.readyState !== WebSocket.OPEN) {
        wsRef.current = null;
      }

      if (gpsStatus.lat && gpsStatus.lon) {
        setDronePath([{ lat: gpsStatus.lat, lng: gpsStatus.lon }]);
      }
    } catch (err) {
      console.error("Failed to restart TerraSLAM:", err);
      // Roll back the optimistic state so the button reflects reality.
      setPendingRecording(null);
    } finally {
      setIsBusy(false);
    }
  }, [gpsStatus, projectId, saveFrames, isBusy, isRecording]);

  const stopRecording = useCallback(async () => {
    if (isBusy) return;

    setIsBusy(true);
    setPendingRecording(false);

    if (recordingIntervalRef.current) {
      clearInterval(recordingIntervalRef.current);
      recordingIntervalRef.current = null;
    }

    try {
      await controlTerraSLAMComponent("all", "stop", projectId);

      if (wsRef.current && wsRef.current.readyState === WebSocket.OPEN) {
        wsRef.current.close();
      }
    } catch (err) {
      console.error("Failed to stop TerraSLAM:", err);
      setPendingRecording(null);
    } finally {
      setIsBusy(false);
    }
  }, [projectId, isBusy]);

  const setTelemetryStatus = useCallback(
    (status: "idle" | "recording" | "active") =>
      setTelemetry((prev) => ({ ...prev, status })),
    [],
  );

  const calibration = useCalibrationSession({
    projectId,
    project,
    systemStatus,
    refetch,
    setTelemetryStatus,
  });

  return {
    project,
    isLoading,
    error,
    isRecording,
    isBusy,
    dronePosition,
    dronePath,
    telemetry,
    showCalibration,
    hasVideoStream,
    videoCanvasRef,
    startRecording,
    stopRecording,
    saveFrames,
    setSaveFrames,
    gpsStatus,
    systemStatus,
    refetch,
    ...calibration,
  };
}
