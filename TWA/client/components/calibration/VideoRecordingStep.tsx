import { useEffect, useRef, useState } from "react";

interface VideoRecordingStepProps {
  onComplete: () => void;
  onBack: () => void;
  onSkip?: () => void;
  recordingStatus: "idle" | "recording" | "stopped";
  recordingDuration: number;
  onRecordingDurationChange: (duration: number) => void;
  onRecordingStatusChange: (status: "idle" | "recording" | "stopped") => void;
  sessionId?: string | null;
  onSessionIdChange?: (id: string) => void;
}

export function VideoRecordingStep({
  onComplete,
  onBack,
  onSkip,
  recordingStatus,
  recordingDuration,
  onRecordingDurationChange,
  onRecordingStatusChange,
  sessionId,
  onSessionIdChange,
}: VideoRecordingStepProps) {
  const mediaRecorderRef = useRef<MediaRecorder | null>(null);
  const chunksRef = useRef<Blob[]>([]);
  const timerRef = useRef<ReturnType<typeof setInterval> | null>(null);
  const videoRef = useRef<HTMLVideoElement | null>(null);
  const canvasRef = useRef<HTMLCanvasElement | null>(null);
  const [isRecording, setIsRecording] = useState(false);
  const [stream, setStream] = useState<MediaStream | null>(null);
  const [cameraError, setCameraError] = useState<string | null>(null);
  const durationRef = useRef(recordingDuration);
  durationRef.current = recordingDuration;

  const MAX_DURATION = 900; // 15 minutes

  const initStream = async (): Promise<MediaStream | null> => {
    try {
      const mediaStream = await navigator.mediaDevices.getUserMedia({
        video: { width: 1280, height: 720 },
        audio: false,
      });
      setStream(mediaStream);
      setCameraError(null);
      if (videoRef.current) {
        videoRef.current.srcObject = mediaStream;
      }
      return mediaStream;
    } catch (err) {
      console.error("Failed to get camera stream:", err);
      setCameraError("Камера недоступна в этом окружении. Запись невозможна.");
      return null;
    }
  };

  const startRecording = async () => {
    try {
      let currentSessionId = sessionId;
      if (!currentSessionId) {
        const stored = localStorage.getItem("calib_session_id");
        if (stored) {
          currentSessionId = stored;
          onSessionIdChange?.(stored);
        } else {
          const res = await fetch("/api/v1/calibration/video/start", { method: "POST" });
          const data = await res.json();
          if (!data.success) throw new Error(data.detail || "Failed to start session");
          currentSessionId = data.session.id;
          onSessionIdChange?.(currentSessionId);
          localStorage.setItem("calib_session_id", currentSessionId);
        }
      }

      let currentStream = stream;
      if (!currentStream) {
        currentStream = await initStream();
      }

      if (!currentStream) {
        console.error("No video stream available");
        return;
      }

      chunksRef.current = [];
      const mediaRecorder = new MediaRecorder(currentStream, { mimeType: "video/webm" });
      mediaRecorderRef.current = mediaRecorder;

      mediaRecorder.ondataavailable = async (event) => {
        if (event.data.size > 0 && currentSessionId) {
          const formData = new FormData();
          formData.append("chunk", event.data, "chunk.webm");
          await fetch(`/api/v1/calibration/video/${currentSessionId}/chunk`, {
            method: "POST",
            body: formData,
          });
        }
      };

      mediaRecorder.start(1000);
      setIsRecording(true);
      onRecordingStatusChange("recording");
      onRecordingDurationChange(0);

      timerRef.current = setInterval(() => {
        const next = durationRef.current + 1;
        if (next >= MAX_DURATION) {
          stopRecording();
          onRecordingDurationChange(MAX_DURATION);
        } else {
          onRecordingDurationChange(next);
        }
      }, 1000);
    } catch (err) {
      console.error("Failed to start recording:", err);
    }
  };

  const stopRecording = async () => {
    if (timerRef.current) {
      clearInterval(timerRef.current);
      timerRef.current = null;
    }

    if (mediaRecorderRef.current && mediaRecorderRef.current.state !== "inactive") {
      mediaRecorderRef.current.stop();
    }

    if (sessionId) {
      await fetch(`/api/v1/calibration/video/${sessionId}/stop`, { method: "POST" });
    }

    setIsRecording(false);
    onRecordingStatusChange("stopped");
  };

  useEffect(() => {
    return () => {
      if (timerRef.current) clearInterval(timerRef.current);
      if (stream) {
        stream.getTracks().forEach((track) => track.stop());
      }
    };
  }, [stream]);

  const formatTime = (seconds: number) => {
    const mins = Math.floor(seconds / 60);
    const secs = seconds % 60;
    return `${mins.toString().padStart(2, "0")}:${secs.toString().padStart(2, "0")}`;
  };

  return (
    <div className="flex flex-col h-full">
      <div className="flex-1 flex items-center justify-center p-6">
        <div className="max-w-2xl w-full space-y-6">
          <div className="text-center">
            <h3 className="text-2xl font-bold text-foreground mb-2">
              Запись видео для калибровки
            </h3>
            <p className="text-muted-foreground">
              Запишите видео до {MAX_DURATION / 60} минут. Держите дрон стабильно.
            </p>
          </div>

          <div className="aspect-video bg-black rounded-lg overflow-hidden">
            <video ref={videoRef} autoPlay playsInline muted className="w-full h-full object-contain" />
          </div>

          {cameraError && (
            <div className="p-3 rounded-md border border-red-300 bg-red-50 text-red-700 text-sm">
              {cameraError}
            </div>
          )}

          <div className="text-center">
            <div className="text-4xl font-mono font-bold text-foreground mb-4">
              {formatTime(recordingDuration)}
            </div>
            <div className="flex justify-center gap-4">
              {!isRecording ? (
                <button
                  onClick={startRecording}
                  className="px-8 py-3 bg-red-600 text-white rounded-lg font-bold flex items-center gap-2 hover:bg-red-700 transition-colors"
                >
                  <span className="w-3 h-3 bg-white rounded-full" />
                  ▶ Записать
                </button>
              ) : (
                <button
                  onClick={stopRecording}
                  className="px-8 py-3 bg-slate-700 text-white rounded-lg font-bold flex items-center gap-2 hover:bg-slate-800 transition-colors"
                >
                  ■ Стоп
                </button>
              )}
              <button onClick={onBack} className="px-6 py-3 border border-border rounded-lg hover:bg-slate-50 transition-colors">
                Отмена
              </button>
              {onSkip && !isRecording && (
                <button onClick={onSkip} className="px-6 py-3 border border-slate-300 text-slate-600 rounded-lg hover:bg-slate-50 transition-colors">
                  Пропустить →
                </button>
              )}
            </div>
            {recordingStatus === "stopped" && (
              <div className="mt-4">
                <button onClick={onComplete} className="px-6 py-3 bg-primary text-white rounded-lg font-bold hover:bg-primary/90 transition-colors">
                  Далее: Обрезка видео
                </button>
              </div>
            )}
          </div>
        </div>
      </div>
    </div>
  );
}
