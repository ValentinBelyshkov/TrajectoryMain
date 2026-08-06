import { useRef, useState, useEffect } from "react";
import {
  uploadCalibrationVideo,
  startCalibrationSession,
  startCalibrationSessionFromProject,
} from "@/lib/api";
import { StepLayout } from "./StepLayout";
import { ErrorBanner } from "./ErrorBanner";

interface VideoUploadStepProps {
  onComplete: () => void;
  onBack: () => void;
  onSkip?: () => void;
  sessionId: string | null;
  onSessionIdChange: (id: string) => void;
  onRecordingStatusChange: (status: "idle" | "recording" | "stopped") => void;
  onRecordingDurationChange: (duration: number) => void;
  projectId?: string;
  projectVideoFilename?: string | null;
}

export function VideoUploadStep({
  onComplete,
  onBack,
  onSkip,
  sessionId,
  onSessionIdChange,
  onRecordingStatusChange,
  onRecordingDurationChange,
  projectId,
  projectVideoFilename,
}: VideoUploadStepProps) {
  const [uploading, setUploading] = useState(false);
  const [error, setError] = useState<string | null>(null);
  const [usingExisting, setUsingExisting] = useState(false);
  const fileInputRef = useRef<HTMLInputElement | null>(null);

  useEffect(() => {
    if (projectVideoFilename && projectId && !sessionId && !usingExisting) {
      setUsingExisting(true);
      (async () => {
        try {
          const data = await startCalibrationSessionFromProject(projectId);
          if (!data.success) throw new Error("Failed to start session from project video");
          onSessionIdChange(data.session.id);
          onRecordingStatusChange("stopped");
          onRecordingDurationChange(0);
          onComplete();
        } catch (err) {
          console.error("Failed to use existing video:", err);
          setError(err instanceof Error ? err.message : "Ошибка использования существующего видео");
          setUsingExisting(false);
        }
      })();
    }
  }, [projectVideoFilename, projectId, sessionId, onSessionIdChange, onRecordingStatusChange, onRecordingDurationChange, onComplete, usingExisting]);

  const handleFileChange = async (e: React.ChangeEvent<HTMLInputElement>) => {
    const file = e.target.files?.[0];
    if (!file) return;

    setError(null);
    setUploading(true);

    try {
      let currentSessionId = sessionId;
      if (!currentSessionId) {
        const data = await startCalibrationSession();
        if (!data.success) throw new Error("Failed to start session");
        currentSessionId = data.session.id;
        onSessionIdChange(currentSessionId);
      }

      await uploadCalibrationVideo(currentSessionId, file);
      onRecordingStatusChange("stopped");
      onRecordingDurationChange(0);
      onComplete();
    } catch (err) {
      console.error("Failed to upload video:", err);
      setError(err instanceof Error ? err.message : "Ошибка загрузки видео");
    } finally {
      setUploading(false);
    }
  };

  if (usingExisting) {
    return (
      <StepLayout>
        <div className="text-center">
          <h3 className="text-2xl font-bold text-foreground mb-2">
            Используется существующее видео
          </h3>
          <p className="text-muted-foreground">
            Видео из проекта: {projectVideoFilename}
          </p>
        </div>
        <div className="flex justify-center">
          <div className="animate-spin rounded-full h-12 w-12 border-b-2 border-primary"></div>
        </div>
      </StepLayout>
    );
  }

  return (
    <StepLayout>
      <div className="text-center">
        <h3 className="text-2xl font-bold text-foreground mb-2">
          Загрузка видео для калибровки
        </h3>
        <p className="text-muted-foreground">
          Выберите готовое видеофайл с записи дрона.
        </p>
      </div>

      <div className="border-2 border-dashed border-border rounded-lg p-12 text-center">
        <input
          ref={fileInputRef}
          type="file"
          accept="video/*"
          onChange={handleFileChange}
          className="hidden"
        />
        <button
          onClick={() => fileInputRef.current?.click()}
          disabled={uploading}
          className="px-8 py-3 bg-primary text-white rounded-lg font-bold hover:bg-primary/90 transition-colors disabled:opacity-50"
        >
          {uploading ? "Загрузка..." : "Выбрать видеофайл"}
        </button>
        <p className="text-sm text-muted-foreground mt-4">
          Поддерживаются форматы MP4, WebM, MOV
        </p>
      </div>

      <ErrorBanner message={error} />

      <div className="flex justify-center gap-4">
        <button onClick={onBack} className="px-6 py-3 border border-border rounded-lg hover:bg-slate-50 transition-colors">
          Отмена
        </button>
        {onSkip && (
          <button onClick={onSkip} className="px-6 py-3 border border-slate-300 text-slate-600 rounded-lg hover:bg-slate-50 transition-colors">
            Пропустить →
          </button>
        )}
      </div>
    </StepLayout>
  );
}
