import { useEffect, useRef, useState, type RefObject } from "react";
import { StepLayout } from "./StepLayout";
import { ErrorBanner } from "./ErrorBanner";
import { checkProjectHasFrames } from "@/lib/api";

interface CameraRecordingStepProps {
  onStart: () => void;
  onStop: () => void;
  videoCanvasRef: RefObject<HTMLCanvasElement | null>;
  hasVideoStream: boolean;
  calibrationSessionId: string | null;
  startSession: () => Promise<{ id: string }>;
  onComplete: () => void;
  onBack: () => void;
  onSkip?: () => void;
  error?: string | null;
  projectId?: string;
  onStartPublisher?: (projectId: string) => Promise<void>;
}

export function CameraRecordingStep({
  onStart,
  onStop,
  videoCanvasRef,
  hasVideoStream,
  calibrationSessionId,
  startSession,
  onComplete,
  onBack,
  error,
  projectId,
  onStartPublisher,
  onSkip,
}: CameraRecordingStepProps) {
  const [isRecording, setIsRecording] = useState(false);
  const [seconds, setSeconds] = useState(0);
  const timerRef = useRef<number | null>(null);

  // Показываем кнопку «Пропустить», только если в папке frames проекта
  // уже есть сохранённые кадры — тогда запись не обязательна.
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

  const start = async () => {
    // Создаём калибровочную сессию (нужна для шага обработки), если ещё нет
    if (!calibrationSessionId) {
      try {
        await startSession();
      } catch (err) {
        console.error("Failed to start calibration session:", err);
      }
    }
    // Запускаем publisher_realsense с папкой procframe проекта (пишет кадры на диск)
    if (projectId && onStartPublisher) {
      try {
        await onStartPublisher(projectId);
      } catch (err) {
        console.error("Failed to start camera publisher:", err);
      }
    }
    setSeconds(0);
    onStart(); // бэкенд: запуск записи с камеры RealSense
    setIsRecording(true);
    timerRef.current = window.setInterval(() => setSeconds((s) => s + 1), 1000);
  };

  const stop = () => {
    if (timerRef.current !== null) {
      window.clearInterval(timerRef.current);
      timerRef.current = null;
    }
    onStop(); // бэкенд: остановка записи
    setIsRecording(false);
  };

  useEffect(() => {
    return () => {
      if (timerRef.current !== null) window.clearInterval(timerRef.current);
    };
  }, []);

  const formatTime = (s: number) => {
    const m = Math.floor(s / 60);
    const sec = s % 60;
    return `${m.toString().padStart(2, "0")}:${sec.toString().padStart(2, "0")}`;
  };

  return (
    <StepLayout>
      {error && (
        <ErrorBanner message={error} />
      )}
      <div className="text-center">
        <h3 className="text-2xl font-bold text-foreground mb-2">
          Запись с камеры дрона (RealSense)
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
            <p className="text-white/60 text-sm">Нет видеопотока с камеры</p>
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
