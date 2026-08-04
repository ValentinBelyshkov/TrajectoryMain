import { useEffect, useRef, useState } from "react";
import { Video, VideoOff, Maximize2, Minimize2 } from "lucide-react";

interface VideoFeedProps {
  projectId: string;
  isSimulation?: boolean;
  isRecording?: boolean;
}

export function VideoFeed({
  projectId,
  isSimulation = false,
  isRecording = false,
}: VideoFeedProps) {
  const videoRef = useRef<HTMLVideoElement>(null);
  const [isFullscreen, setIsFullscreen] = useState(false);
  const [error, setError] = useState<string | null>(null);

  useEffect(() => {
    if (isSimulation && videoRef.current) {
      // For simulation, we would load the uploaded video
      // In production, this would connect to WebSocket or MJPEG stream
    }
  }, [projectId, isSimulation]);

  const toggleFullscreen = () => {
    if (!document.fullscreenElement) {
      videoRef.current?.parentElement?.requestFullscreen();
      setIsFullscreen(true);
    } else {
      document.exitFullscreen();
      setIsFullscreen(false);
    }
  };

  return (
    <div className="relative w-full h-full bg-black rounded-lg overflow-hidden">
      {/* Video element or placeholder */}
      <div className="absolute inset-0 flex items-center justify-center">
        {isSimulation ? (
          <video
            ref={videoRef}
            className="w-full h-full object-contain"
            controls={false}
            onError={() => setError("Не удалось загрузить видео")}
          >
            <source src={`/api/video/stream/${projectId}`} type="video/mp4" />
          </video>
        ) : (
          <div className="text-center">
            <Video className="w-16 h-16 text-white/30 mx-auto mb-4" />
            <p className="text-white/50 text-sm">Видеопоток с камеры дрона</p>
            {error && <p className="text-red-400 text-sm mt-2">{error}</p>}
          </div>
        )}
      </div>

      {/* Recording indicator */}
      {isRecording && (
        <div className="absolute top-4 left-4 flex items-center gap-2 bg-red-500/90 px-3 py-1.5 rounded-full">
          <div className="w-2.5 h-2.5 bg-white rounded-full animate-pulse" />
          <span className="text-white text-sm font-medium">REC</span>
        </div>
      )}

      {/* Overlay controls */}
      <div className="absolute bottom-4 right-4 flex gap-2">
        <button
          onClick={toggleFullscreen}
          className="p-2 bg-black/50 hover:bg-black/70 rounded-lg transition-colors"
          title={
            isFullscreen
              ? "Выйти из полноэкранного режима"
              : "Полноэкранный режим"
          }
        >
          {isFullscreen ? (
            <Minimize2 className="w-5 h-5 text-white" />
          ) : (
            <Maximize2 className="w-5 h-5 text-white" />
          )}
        </button>
      </div>

      {/* Camera info overlay */}
      <div className="absolute top-4 right-4 flex items-center gap-2 bg-black/50 px-3 py-1.5 rounded-lg">
        <div
          className={`w-2 h-2 rounded-full ${isRecording ? "bg-red-500" : "bg-green-500"}`}
        />
        <span className="text-white text-xs">
          {isSimulation ? "Симуляция" : "Камера"}
        </span>
      </div>
    </div>
  );
}
