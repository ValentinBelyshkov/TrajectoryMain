import { useRef, useState, useEffect } from "react";
import { uploadCalibrationVideo } from "@/lib/api";

interface VideoUploadStepProps {
  onComplete: () => void;
  onBack: () => void;
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
          const res = await fetch(`/api/v1/calibration/video/start-from-project/${projectId}`, { method: "POST" });
          const data = await res.json();
          if (!data.success) throw new Error(data.detail || "Failed to start session from project video");
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
        const res = await fetch("/api/v1/calibration/video/start", { method: "POST" });
        const data = await res.json();
        if (!data.success) throw new Error(data.detail || "Failed to start session");
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
      <div className="flex flex-col h-full">
        <div className="flex-1 flex items-center justify-center p-6">
          <div className="max-w-2xl w-full space-y-6">
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
          </div>
        </div>
      </div>
    );
  }

  return (
    <div className="flex flex-col h-full">
      <div className="flex-1 flex items-center justify-center p-6">
        <div className="max-w-2xl w-full space-y-6">
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

          {error && (
            <div className="p-3 rounded-md border border-red-300 bg-red-50 text-red-700 text-sm">
              {error}
            </div>
          )}

          <div className="flex justify-center gap-4">
            <button onClick={onBack} className="px-6 py-3 border border-border rounded-lg hover:bg-slate-50 transition-colors">
              Отмена
            </button>
          </div>
        </div>
      </div>
    </div>
  );
}
