import { useState, useRef, useCallback, useReducer, useEffect } from "react";
import {
  uploadCalibrationImage,
  downloadGeotiff,
  matchImageToGeotiff,
  getAutoCalibrationFrames,
  type AutoCalibrationRegion,
  type Project,
  startCalibrationSession,
  startCalibrationSessionFromProject,
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
  const calibrationSessionRef = useRef(calibrationSession);
  calibrationSessionRef.current = calibrationSession;
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
        // Only auto-jump to "complete" from the neutral state. Otherwise a
        // background project refetch (or a restored ?calib=... step) would
        // kick the user out of an in-progress re-calibration.
        if (calibrationStep === "idle") {
          setCalibrationStep("complete");
        }
      } else if (calibrationStep === "complete") {
        setCalibrationStep("idle");
      }
    }
  }, [project, isCalibrated, calibrationStep]); // eslint-disable-line react-hooks/exhaustive-deps

  const handleCalibrate = useCallback(() => {
    setCalibrationStep("type-selection");
  }, []);

  const handleCalibrationTypeSelect = useCallback(async (type: "manual" | "auto") => {
    if (type === "auto") {
      setCalibrationStep("auto-region");
      return;
    }

    // Manual calibration: if the project already has a video (uploaded at
    // creation time), skip the upload/recording step and go straight to the
    // video editor (trimming) using the existing project video.
    if (project?.videoFilename && projectId) {
      try {
        const data = await startCalibrationSessionFromProject(projectId);
        if (data.success) {
          setCalibrationSessionId(data.session.id);
          setCalibrationSession(data.session);
          localStorage.setItem("calib_session_id", data.session.id);
          setCalibrationStep("trimming");
          return;
        }
      } catch (err) {
        console.error("Failed to start session from project video:", err);
      }
    }

    setCalibrationStep("recording");
  }, [project, projectId]);

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
    try {
      localStorage.setItem(`calib_session_id:${projectId}`, data.session.id);
    } catch {
      // ignore storage errors
    }
    return data.session;
  }, [projectId]);

  const resumeCalibrationSession = useCallback(async (sessionId: string) => {
    const data = await getCalibrationSession(sessionId);
    if (!data.success) throw new Error("Failed to load session");
    setCalibrationSessionId(sessionId);
    setCalibrationSession(data.session);
    return data.session;
  }, []);

  // Keep the session in sync when arriving at the correlating step (e.g. after
  // skipping earlier steps). Without this, c.calibrationSession could be stale
  // with an empty frame_pose_data, so CorrelationStep would fall back to the
  // pose-less procframe list and every point would reject.
  useEffect(() => {
    if (calibrationStep === "correlating" && calibrationSessionId) {
      resumeCalibrationSession(calibrationSessionId).catch((err) =>
        console.error("Failed to refresh calibration session:", err),
      );
    }
  }, [calibrationStep, calibrationSessionId, resumeCalibrationSession]);

  const handleStartRecording = useCallback(async () => {
    try {
      setRecordingError(null);
      // The session id is owned by the server. Trust localStorage (namespaced
      // per project) for the *current* project, not the stale in-memory
      // calibrationSession captured in this closure, otherwise a resume() on an
      // unrelated/stale id could start the UI pointing at the wrong session.
      let session = calibrationSessionRef.current;
      const storedKey = projectId ? `calib_session_id:${projectId}` : null;
      const stored = storedKey ? localStorage.getItem(storedKey) : null;
      if (!session) {
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
      // A session loaded from the URL/localStorage may already be in trimming,
      // processing, or another terminal state. It cannot be restarted by
      // merely changing the React state; create a fresh recording session.
      if (!session || session.status !== "recording") {
        session = await startNewCalibrationSession();
      }
      setRecordingStatus("recording");
      setCalibrationStep("recording");
      setRecordingDuration(0);
    } catch (err) {
      const msg = err instanceof Error ? err.message : "Failed to start recording session";
      setRecordingError(msg);
      console.error("Failed to start recording session:", err);
    }
  }, [projectId, calibrationSessionRef, startNewCalibrationSession, resumeCalibrationSession]);

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

      // Start a FRESH calibration session from the (now trimmed) project
      // frames. This resets the session to a processable state (trimming,
      // empty frame_pose_data) so that clicking «Обработать» actually runs
      // SLAM instead of being rejected with "not in processable state" when
      // reusing an already-correlating session.
      try {
        const data = await startCalibrationSessionFromProject(projectId);
        if (data.success) {
          setCalibrationSessionId(data.session.id);
          setCalibrationSession(data.session);
          localStorage.setItem(`calib_session_id:${projectId}`, data.session.id);
        }
      } catch (err) {
        console.error("Failed to start fresh session after trim:", err);
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

    // The session we actually drive. May be swapped to a freshly created one
    // below (when re-running on an already-processed session).
    let activeSessionId = calibrationSessionId;

    try {
      // «Обработать» always attempts to run SLAM. If the current session is
      // already processed (correlating/done) the backend refuses with
      // "not in processable state" — in that case we start a FRESH session
      // from the project's (already trimmed) frames and retry, so processing
      // actually runs instead of being silently skipped.
      try {
        const startData = await processCalibrationSession(
          activeSessionId,
          projectId,
        );
        if (!startData.success) throw new Error("Не удалось запустить обработку");
      } catch (startErr) {
        const msg = startErr instanceof Error ? startErr.message : "";
        if (msg.includes("not in processable state") && projectId) {
          const fresh = await startCalibrationSessionFromProject(projectId);
          if (fresh.success) {
            activeSessionId = fresh.session.id;
            setCalibrationSessionId(fresh.session.id);
            setCalibrationSession(fresh.session);
            localStorage.setItem(`calib_session_id:${projectId}`, fresh.session.id);
            const startData = await processCalibrationSession(
              fresh.session.id,
              projectId,
            );
            if (!startData.success)
              throw new Error("Не удалось запустить обработку");
          } else {
            throw startErr;
          }
        } else {
          throw startErr;
        }
      }

      const deadline = Date.now() + 30 * 60 * 1000;

      while (true) {
        if (Date.now() > deadline) {
          throw new Error("Превышено время ожидания обработки SLAM");
        }

        await new Promise((resolve) => setTimeout(resolve, 1500));

        // Poll the AUTHORITATIVE session status. The in-memory progress
        // tracker does not carry session_status when a run is in flight, so
        // we read the real session instead of relying on prog.session_status.
        let status: string | null = null;
        try {
          const sess = await getCalibrationSession(activeSessionId);
          setCalibrationSession(sess.session);
          status = sess.session.status;
        } catch {
          // session not readable yet — fall through to progress text
        }

        if (status === "correlating" || status === "done") {
          setCalibrationStep("correlating");
          return;
        }

        if (status === "error") {
          const prog = await getCalibrationProgress(activeSessionId).catch(
            () => null,
          );
          throw new Error(prog?.error || "Ошибка при обработке SLAM");
        }

        const prog = await getCalibrationProgress(activeSessionId).catch(
          () => null,
        );
        if (prog?.found) {
          const label = prog.step_label || "Обработка...";
          const frames = prog.frames_total
            ? ` (${prog.frames_done ?? 0}/${prog.frames_total})`
            : "";
          const elapsed = prog.elapsed != null ? ` — ${prog.elapsed} с` : "";
          setProcessingProgress(`${label}${frames}${elapsed}`);
        }
      }
    } catch (err) {
      setProcessingProgress(
        err instanceof Error ? err.message : "Ошибка обработки",
      );
      console.error("Processing failed:", err);
    }
  }, [calibrationSessionId, projectId]);

  // "Пропустить" on the processing step must advance to the next screen
  // (correlating), never re-run SLAM. The correlating step refreshes the
  // session data via its own resume effect, so any collected frames/poses are
  // picked up there. Re-running SLAM here was the cause of
  // "пропустить перезапускает обработку".
  const handleSkipProcessing = useCallback(async () => {
    if (calibrationSessionId) {
      resumeCalibrationSession(calibrationSessionId).catch((err) =>
        console.error("Failed to refresh calibration session:", err),
      );
    }
    setCalibrationStep("correlating");
  }, [calibrationSessionId, resumeCalibrationSession]);

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
    localStorage.removeItem(`calib_session_id:${projectId}`);
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
    handleSkipProcessing,
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

