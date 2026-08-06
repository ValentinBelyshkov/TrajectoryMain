import { useState, useRef, useCallback, useReducer, useEffect } from "react";
import {
  uploadCalibrationImage,
  downloadGeotiff,
  matchImageToGeotiff,
  getAutoCalibrationFrames,
  type AutoCalibrationRegion,
  type Project,
  startCalibrationSession,
  stopCalibrationSession,
  getCalibrationSession,
  trimCalibrationVideo,
  processCalibrationSession,
  getCalibrationProgress,
  correlateCalibrationPoints,
  finalizeCalibrationSession,
  trimProjectFrames,
} from "@/lib/api";
import type { CalibrationPoint } from "@/components/CalibrationPointSelector";
import type { CalibrationStep } from "./useProject";
import type { TerraSLAMStatus } from "./useTerraSLAMStatus";

interface UseCalibrationSessionArgs {
  projectId: string | undefined;
  project: Project | undefined;
  systemStatus: TerraSLAMStatus | null;
  refetch: () => void;
  setTelemetryStatus: (status: "idle" | "recording" | "active") => void;
}

export function useCalibrationSession({
  projectId,
  project,
  systemStatus,
  refetch,
  setTelemetryStatus,
}: UseCalibrationSessionArgs) {
  const [calibrationStep, dispatchStep] = useReducer(
    (_: CalibrationStep, step: CalibrationStep) => step,
    "idle",
  );
  const setCalibrationStep = useCallback(
    (step: CalibrationStep) => dispatchStep(step),
    [],
  );

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

  // Auto calibration state
  const [autoCalibrationRegion, setAutoCalibrationRegion] =
    useState<AutoCalibrationRegion | null>(null);
  const [autoCalibrationFrames, setAutoCalibrationFrames] = useState<
    { filename: string; url: string }[]
  >([]);
  const [autoCalibrationError, setAutoCalibrationError] = useState<string | null>(
    null,
  );
  const [autoCalibrationProgress, setAutoCalibrationProgress] = useState<
    "idle" | "downloading" | "matching" | "success" | "error"
  >("idle");
  const [autoCalibrationMessage, setAutoCalibrationMessage] = useState<
    string | null
  >(null);

  // Calibration session state
  const [calibrationSessionId, setCalibrationSessionId] = useState<string | null>(
    null,
  );
  const [calibrationSession, setCalibrationSession] = useState<any>(null);
  const [recordingStatus, setRecordingStatus] = useState<
    "idle" | "recording" | "stopped"
  >("idle");
  const [recordingDuration, setRecordingDuration] = useState(0);
  const [trimSegments, setTrimSegments] = useState<
    Array<{ start: number; end: number }>
  >([]);
  const [processingProgress, setProcessingProgress] = useState<string>("");
  const [correlationPoints, setCorrelationPoints] = useState<any[]>([]);
  const [transform, setTransform] = useState<any>(null);
  const [recordingError, setRecordingError] = useState<string | null>(null);

  const isCalibrated = project?.calibrationStatus === "calibrated";

  useEffect(() => {
    if (project) {
      if (isCalibrated) {
        setCalibrationStep("complete");
      } else if (calibrationStep === "complete") {
        setCalibrationStep("idle");
      }
    }
  }, [project, isCalibrated]); // eslint-disable-line react-hooks/exhaustive-deps

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

  const handleFramesSelected = useCallback(
    (frames: { filename: string; url: string }[]) => {
      setSelectedFrames(frames);
      setCalibrationStep("pairing");
    },
    [],
  );

  // Auto calibration handlers
  const handleAutoRegionConfirm = useCallback(
    async (region: AutoCalibrationRegion) => {
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

          const frames = await getAutoCalibrationFrames(projectId);
          setAutoCalibrationFrames(frames);
        } else {
          setAutoCalibrationError(
            result.error || result.message || "Ошибка загрузки карты",
          );
          setAutoCalibrationProgress("error");
          setCalibrationStep("auto-region");
        }
      } catch (err) {
        setAutoCalibrationError(
          err instanceof Error ? err.message : "Ошибка загрузки карты",
        );
        setAutoCalibrationProgress("error");
        setCalibrationStep("auto-region");
      }
    },
    [projectId],
  );

  const handleAutoImageSelect = useCallback(
    async (imageFilename: string) => {
      if (!projectId) return;

      setAutoCalibrationProgress("matching");
      setAutoCalibrationMessage("Сопоставление изображения с картой...");
      setAutoCalibrationError(null);

      try {
        const result = await matchImageToGeotiff(projectId, imageFilename);

        if (result.success) {
          setAutoCalibrationProgress("success");
          setAutoCalibrationMessage("Калибровка успешно пройдена!");

          await refetch();
          setCalibrationStep("complete");
        } else {
          setAutoCalibrationError(
            result.message || "Ошибка калибровки. Попробуйте другое фото или параметры снимка",
          );
          setAutoCalibrationProgress("error");
        }
      } catch (err) {
        setAutoCalibrationError(
          err instanceof Error ? err.message : "Ошибка калибровки",
        );
        setAutoCalibrationProgress("error");
      }
    },
    [projectId, refetch],
  );

  const handleAutoCalibrationBack = useCallback(() => {
    setCalibrationStep("type-selection");
    setAutoCalibrationRegion(null);
    setAutoCalibrationFrames([]);
    setAutoCalibrationError(null);
    setAutoCalibrationProgress("idle");
    setAutoCalibrationMessage(null);
  }, []);

  const startNewCalibrationSession = useCallback(async (clearExistingFrames: boolean = true) => {
    const data = await startCalibrationSession(projectId, clearExistingFrames);
    if (!data.success) throw new Error("Failed to start session");
    setCalibrationSessionId(data.session.id);
    setCalibrationSession(data.session);
    localStorage.setItem("calib_session_id", data.session.id);
    return data.session;
  }, [projectId]);

  const resumeCalibrationSession = useCallback(async (sessionId: string) => {
    const data = await getCalibrationSession(sessionId);
    if (!data.success) throw new Error("Failed to load session");
    setCalibrationSessionId(sessionId);
    setCalibrationSession(data.session);
    return data.session;
  }, []);

  const handleStartRecording = useCallback(async () => {
    try {
      setRecordingError(null);
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
    } catch (err) {
      const msg = err instanceof Error ? err.message : "Failed to start recording session";
      setRecordingError(msg);
      console.error("Failed to start recording session:", err);
    }
  }, [calibrationSession, startNewCalibrationSession, resumeCalibrationSession]);

  const handleStopRecording = useCallback(async () => {
    if (calibrationSessionId) {
      try {
        await stopCalibrationSession(calibrationSessionId);
      } catch (err) {
        console.error("Failed to stop recording session:", err);
      }
    }
    setRecordingStatus("stopped");
    // Both camera (frames) and simulation (video) recordings go through a
    // trimming step next. The workflow renders the appropriate editor:
    // frame-based trimming for camera mode, video trimming for simulation.
    setCalibrationStep("trimming");
  }, [calibrationSessionId, project, systemStatus]);

  const handleApplyFrameTrim = useCallback(
    async (keep: string[]) => {
      if (!projectId) {
        setCalibrationStep("processing");
        return;
      }
      try {
        await trimProjectFrames(projectId, keep);
      } catch (err) {
        console.error("Failed to trim frames:", err);
      }
      setCalibrationStep("processing");
    },
    [projectId],
  );

  const handleApplyTrim = useCallback(
    async (segments: Array<{ start: number; end: number }>) => {
      if (!calibrationSessionId) return;
      setTrimSegments(segments);
      const data = await trimCalibrationVideo(calibrationSessionId, segments);
      if (!data.success) throw new Error("Trim failed");
      setCalibrationSession(data.session);
      setCalibrationStep("processing");
    },
    [calibrationSessionId],
  );

  const handleRunProcessing = useCallback(async () => {
    if (!calibrationSessionId || !projectId) return;

    try {
      const existing = await getCalibrationSession(calibrationSessionId);
      if (existing.success && existing.session.status === "correlating") {
        setCalibrationSession(existing.session);
        setCalibrationStep("correlating");
        return;
      }
    } catch {
      // сессия ещё не готова — продолжаем запуск
    }

    try {
      const startData = await processCalibrationSession(
        calibrationSessionId,
        projectId,
      );
      if (!startData.success) throw new Error("Не удалось запустить обработку");

      const deadline = Date.now() + 30 * 60 * 1000;

      while (true) {
        if (Date.now() > deadline) {
          throw new Error("Превышено время ожидания обработки SLAM");
        }

        await new Promise((resolve) => setTimeout(resolve, 1500));

        const prog = await getCalibrationProgress(calibrationSessionId);

        if (!prog.found) {
          if (
            prog.session_status === "correlating" ||
            prog.session_status === "done"
          ) {
            const sess = await getCalibrationSession(calibrationSessionId);
            setCalibrationSession(sess.session);
            setCalibrationStep("correlating");
            return;
          }
          continue;
        }

        const label = prog.step_label || "Обработка...";
        const frames = prog.frames_total
          ? ` (${prog.frames_done ?? 0}/${prog.frames_total})`
          : "";
        const elapsed = prog.elapsed != null ? ` — ${prog.elapsed} с` : "";
        setProcessingProgress(`${label}${frames}${elapsed}`);

        if (prog.step === "error" || prog.slam_crashed) {
          throw new Error(prog.error || "Ошибка при обработке SLAM");
        }

        if (prog.step === "done") {
          const sess = await getCalibrationSession(calibrationSessionId);
          setCalibrationSession(sess.session);
          setCalibrationStep("correlating");
          return;
        }
      }
    } catch (err) {
      setProcessingProgress(
        err instanceof Error ? err.message : "Ошибка обработки",
      );
      console.error("Processing failed:", err);
    }
  }, [calibrationSessionId, projectId]);

  const handleComputeCorrelation = useCallback(
    async (points: any[]) => {
      if (!calibrationSessionId) return;
      setCorrelationPoints(points);
      const data = await correlateCalibrationPoints(calibrationSessionId, points);
      if (!data.success) throw new Error("Correlation failed");
      setTransform(data.transform);
      setCalibrationSession(data.session);
      setCalibrationStep("finalizing");
    },
    [calibrationSessionId],
  );

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
          err instanceof Error ? err.message : "Ошибка загрузки изображения",
        );
      } finally {
        setIsUploading(false);
      }
    },
    [projectId],
  );

  const handleCalibrationComplete = useCallback(async () => {
    if (!projectId) return;

    try {
      await refetch();
      setCalibrationStep("complete");
      setTelemetryStatus("active");
    } catch (err) {
      alert(err instanceof Error ? err.message : "Ошибка обновления данных");
    }
  }, [projectId, refetch, setTelemetryStatus]);

  const handleCalibrationCancel = useCallback(() => {
    setCalibrationStep(
      project?.calibrationStatus === "calibrated" ? "complete" : "idle",
    );
    setUploadedImage(null);
    setSelectedFrames([]);
  }, [project?.calibrationStatus]);

  const clearUploadError = useCallback(() => {
    setUploadError(null);
  }, []);

  return {
    calibrationStep,
    setCalibrationStep,
    uploadedImage,
    selectedFrames,
    uploadError,
    isUploading,
    autoCalibrationRegion,
    autoCalibrationFrames,
    autoCalibrationError,
    autoCalibrationProgress,
    autoCalibrationMessage,
    handleAutoRegionConfirm,
    handleAutoImageSelect,
    handleAutoCalibrationBack,
    calibrationSessionId,
    setCalibrationSessionId,
    calibrationSession,
    recordingStatus,
    setRecordingStatus,
    recordingError,
    recordingDuration,
    setRecordingDuration,
    trimSegments,
    processingProgress,
    correlationPoints,
    transform,
    handleStartRecording,
    handleStopRecording,
    handleApplyTrim,
    handleApplyFrameTrim,
    handleRunProcessing,
    handleComputeCorrelation,
    handleFinalizeCalibration,
    startNewCalibrationSession,
    resumeCalibrationSession,
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
  };
}
