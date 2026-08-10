import { useEffect, useRef, useState, type RefObject } from "react";
import { StepLayout } from "./StepLayout";
import { ErrorBanner } from "./ErrorBanner";
import { checkProjectHasFrames, getCalibrationRecordingStatus } from "@/lib/api";

interface CameraRecordingStepProps {
  onStart: () => void | Promise<void>;
  onStop: () => void | Promise<void>;
  videoCanvasRef: RefObject<HTMLCanvasElement | null>;
  hasVideoStream: boolean;
  calibrationSessionId: string | null;
  sessionStatus?: string | null;
  startSession: () => Promise<{ id: string }>;
  onComplete: () => void;
  onBack: () => void;
  onSkip?: () => void;
  error?: string | null;
  projectId?: string;
  onStartPublisher?: (projectId: string) => Promise<void>;
  setCalibrationSessionId?: (id: string) => void;
}

export function CameraRecordingStep({
  onStart,
  onStop,
  videoCanvasRef,
  hasVideoStream,
  calibrationSessionId,
  sessionStatus,
  startSession,
  onComplete,
  onBack,
  onSkip,
  error,
  projectId,
  onStartPublisher,
  setCalibrationSessionId,
}: CameraRecordingStepProps) {
  const [isRecording, setIsRecording] = useState(false);
  const [seconds, setSeconds] = useState(0);
  const timerRef = useRef<number | null>(null);
  // A status request can outlive the click that started/stopped recording.
  // Responses from before the latest user intent must never overwrite the
  // optimistic button state.
  const syncRequestRef = useRef(0);
  const interactionEpochRef = useRef(0);

  const [hasExistingFrames, setHasExistingFrames] = useState(false);
  useEffect(() => {
    if (!projectId) {
      setHasExistingFrames(false);
      return;
    }
    let active = true;
    checkProjectHasFrames(projectId)
      .then((res) => {
        if (active) setHasExistingFrames(res.frame_count > 0);
      })
      .catch(() => {
        if (active) setHasExistingFrames(false);
      });
    return () => {
      active = false;
    };
  }, [projectId]);

  const startTimer = () => {
    if (timerRef.current !== null) return;
    timerRef.current = window.setInterval(() => {
      setSeconds((s) => s + 1);
    }, 1000);
  };

  const stopTimer = () => {
    if (timerRef.current !== null) {
      window.clearInterval(timerRef.current);
      timerRef.current = null;
    }
  };

  useEffect(() => {
    if (!projectId) return;

    let cancelled = false;

    const sync = async () => {
      const requestId = ++syncRequestRef.current;
      const interactionEpoch = interactionEpochRef.current;
      try {
        const status = await getCalibrationRecordingStatus(projectId);
        if (
          cancelled ||
          requestId !== syncRequestRef.current ||
          interactionEpoch !== interactionEpochRef.current
        ) {
          return;
        }
        // The server is now authoritative. Keep the live session id in sync so
        // the Stop button always targets the recording that is actually active
        // (e.g. after a page refresh the URL may still hold a stale id).
        if (status.session_id && status.session_id !== calibrationSessionId) {
          setCalibrationSessionId?.(status.session_id);
          try {
            localStorage.setItem(
              `calib_session_id:${projectId}`,
              status.session_id,
            );
          } catch {
            // ignore storage errors
          }
        }
        // `status` is authoritative when available. Accept the boolean too
        // for compatibility with older backends, but tolerate an inconsistent
        // response such as {status: "recording", recording: false}.
        const recording =
          status.status === "recording" || status.recording === true;
        setIsRecording(recording);
        if (recording) {
          setSeconds(status.elapsed || 0);
          startTimer();
        } else {
          stopTimer();
        }
      } catch {
        if (!cancelled) {
          // Network/backend error: do NOT clobber the current recording state.
          // A transient 502 (backend down) must not flip the button back to
          // "Записать" while a capture process may still be running.
        }
      }
    };

    sync();
    const id = window.setInterval(sync, 5000);
    return () => {
      cancelled = true;
      window.clearInterval(id);
      stopTimer();
    };
  }, [projectId, setCalibrationSessionId]);

  const start = async () => {
    // Invalidate any status request that started before this click.
    interactionEpochRef.current += 1;
    setIsRecording(true);
    setSeconds(0);
    startTimer();

    if (projectId && onStartPublisher) {
      try {
        await onStartPublisher(projectId);
      } catch (err) {
        console.error("Failed to start camera publisher:", err);
      }
    }
    try {
      await onStart();
    } catch (err) {
      // The hook reports the error itself. Keep the optimistic state until a
      // fresh server status confirms that recording did not start.
      console.error("Failed to start recording:", err);
    }
  };

  const stop = async () => {
    // Invalidate in-flight status requests so an older `recording: true`
    // response cannot restore the Stop button after this click.
    interactionEpochRef.current += 1;
    stopTimer();
    setIsRecording(false);
    try {
      await onStop();
    } catch (err) {
      console.error("Failed to stop recording:", err);
    }
  };

  const formatTime = (s: number) => {
    const m = Math.floor(s / 60);
    const sec = s % 60;
    return `${m.toString().padStart(2, "0")}:${sec.toString().padStart(2, "0")}`;
  };

  return (
    <StepLayout>
      {error && <ErrorBanner message={error} />}
      <div className="text-center">
        <h3 className="text-2xl font-bold text-foreground mb-2">
          Запись с камерой дрона (RealSense)
        </h3>
        <p className="text-muted-foreground">
          Запись ведётся с бортовой камеры. Кадры сохраняются на сервере.
        </p>
      </div>

      <div className="aspect-video bg-black rounded-lg overflow-hidden relative">
        <canvas
          ref={videoCanvasRef}
          className={`absolute inset-0 w-full h-full object-contain ${
            hasVideoStream ? "opacity-100" : "opacity-0"
          }`}
          style={{ imageRendering: "pixelated" }}
        />
        {!hasVideoStream && (
          <div className="absolute inset-0 flex items-center justify-center bg-gray-900">
            <p className="text-white/60 text-sm">Нет видеопотока с камерой</p>
          </div>
        )}
        {isRecording && (
          <div className="absolute top-4 right-4 flex items-center gap-2 bg-red-500 text-white px-3 py-1 rounded-full animate-pulse z-10">
            <div className="w-2 h-2 bg-white rounded-full" />
            <span className="font-semibold text-sm">Запись</span>
          </div>
        )}
      </div>

      <div className="text-center">
          <div className="text-4xl font-mono font-bold text-foreground mb-4">
            {formatTime(seconds)}
          </div>
        <div className="flex justify-center gap-4">
          {!isRecording ? (
            <button
              onClick={start}
              className="px-8 py-3 bg-red-600 text-white rounded-lg font-bold flex items-center gap-2 hover:bg-red-700 transition-colors"
            >
              <span className="w-3 h-3 bg-white rounded-full" />
              ▶ Записать
            </button>
          ) : (
            <button
              onClick={stop}
              className="px-8 py-3 bg-slate-700 text-white rounded-lg font-bold flex items-center gap-2 hover:bg-slate-800 transition-colors"
            >
              ■ Стоп
            </button>
          )}
          <button
            onClick={onBack}
            className="px-6 py-3 border border-border rounded-lg hover:bg-slate-50 transition-colors"
          >
            Отмена
          </button>
        </div>
        {!isRecording && hasExistingFrames && onSkip && (
          <div className="mt-4">
            <button
              onClick={onSkip}
              className="px-6 py-3 border border-slate-300 text-slate-600 rounded-lg hover:bg-slate-50 transition-colors"
            >
              Пропустить запись (кадры уже есть) →
            </button>
          </div>
        )}
        {!isRecording && seconds > 0 && (
          <div className="mt-4">
            <button
              onClick={onComplete}
              className="px-6 py-3 bg-primary text-white rounded-lg font-bold hover:bg-primary/90 transition-colors"
            >
              Далее: Обработка SLAM
            </button>
          </div>
        )}
      </div>
    </StepLayout>
  );
}

