import { useEffect, useRef, useState, type RefObject } from "react";
import { StepLayout } from "./StepLayout";
import { ErrorBanner } from "./ErrorBanner";
import { checkProjectHasFrames, getCalibrationRecordingStatus } from "@/lib/api";

interface CameraRecordingStepProps {
  onStart: () => void;
  onStop: () => void;
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
      try {
        const status = await getCalibrationRecordingStatus(projectId);
        if (cancelled) return;
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
        setIsRecording(status.recording);
        if (status.recording) {
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
  }, [projectId, calibrationSessionId, sessionStatus, setCalibrationSessionId]);

  const start = async () => {
    if (!calibrationSessionId) {
      try {
        await startSession();
      } catch (err) {
        console.error("Failed to start calibration session:", err);
      }
    }
    if (projectId && onStartPublisher) {
      try {
        await onStartPublisher(projectId);
      } catch (err) {
        console.error("Failed to start camera publisher:", err);
      }
    }
    setSeconds(0);
    onStart();
    setIsRecording(true);
    startTimer();
  };

  const stop = () => {
    stopTimer();
    onStop();
    setIsRecording(false);
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
