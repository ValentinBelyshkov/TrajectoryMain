import { useRef, useState, useEffect } from "react";

interface Segment {
  start: number;
  end: number;
}

interface VideoTrimmingStepProps {
  sessionId: string;
  trimmedVideoPath?: string;
  onApplyTrim: (segments: Segment[]) => void;
  onBack: () => void;
}

export function VideoTrimmingStep({ sessionId, trimmedVideoPath, onApplyTrim, onBack }: VideoTrimmingStepProps) {
  const videoRef = useRef<HTMLVideoElement | null>(null);
  const canvasRef = useRef<HTMLCanvasElement | null>(null);
  const [currentTime, setCurrentTime] = useState(0);
  const [duration, setDuration] = useState(0);
  const [segments, setSegments] = useState<Segment[]>([]);
  const [mode, setMode] = useState<"in" | "out">("in");

  const videoUrl = trimmedVideoPath
    ? `/api/v1/calibration/session/${sessionId}/video`
    : `/api/v1/calibration/session/${sessionId}/video`;

  useEffect(() => {
    const video = videoRef.current;
    if (!video) return;
    const handleTimeUpdate = () => setCurrentTime(video.currentTime);
    const handleLoadedMetadata = () => setDuration(video.duration);
    video.addEventListener("timeupdate", handleTimeUpdate);
    video.addEventListener("loadedmetadata", handleLoadedMetadata);
    return () => {
      video.removeEventListener("timeupdate", handleTimeUpdate);
      video.removeEventListener("loadedmetadata", handleLoadedMetadata);
    };
  }, []);

  const addSegment = () => {
    if (!mode) return;
    if (mode === "in") {
      setSegments((prev) => {
        const next = [...prev, { start: currentTime, end: duration }];
        return next;
      });
      setMode("out");
    } else {
      setSegments((prev) => {
        if (prev.length === 0) return prev;
        const next = [...prev];
        next[next.length - 1] = { ...next[next.length - 1], end: currentTime };
        return next;
      });
      setMode("in");
    }
  };

  const drawTimeline = () => {
    const canvas = canvasRef.current;
    if (!canvas) return;
    const ctx = canvas.getContext("2d");
    if (!ctx) return;
    const w = canvas.width;
    const h = canvas.height;
    ctx.fillStyle = "#010409";
    ctx.fillRect(0, 0, w, h);

    if (duration <= 0) return;
    const px = (t: number) => (t / duration) * w;

    ctx.fillStyle = "rgba(88,166,255,0.15)";
    segments.forEach((seg) => {
      ctx.fillRect(px(seg.start), 0, px(seg.end) - px(seg.start), h);
    });

    ctx.strokeStyle = "#58a6ff";
    ctx.lineWidth = 2;
    segments.forEach((seg) => {
      ctx.strokeRect(px(seg.start), 0, px(seg.end) - px(seg.start), h);
    });

    const playX = px(currentTime);
    ctx.strokeStyle = "#f85149";
    ctx.lineWidth = 2;
    ctx.beginPath();
    ctx.moveTo(playX, 0);
    ctx.lineTo(playX, h);
    ctx.stroke();
  };

  useEffect(() => {
    drawTimeline();
  }, [currentTime, duration, segments]);

  const handleCanvasClick = (e: React.MouseEvent<HTMLCanvasElement>) => {
    const canvas = canvasRef.current;
    if (!canvas || duration <= 0) return;
    const rect = canvas.getBoundingClientRect();
    const x = (e.clientX - rect.left) / rect.width;
    const t = x * duration;
    setCurrentTime(t);
    if (videoRef.current) {
      videoRef.current.currentTime = t;
    }
  };

  const handleApply = () => {
    if (segments.length === 0) return;
    onApplyTrim(segments);
  };

  return (
    <div className="flex flex-col h-full">
      <div className="flex-1 overflow-auto p-6">
        <div className="max-w-4xl mx-auto space-y-4">
          <div className="aspect-video bg-black rounded-lg overflow-hidden">
            <video ref={videoRef} controls src={videoUrl} className="w-full h-full object-contain" />
          </div>
          <div className="space-y-2">
            <div className="flex items-center gap-2 flex-wrap">
              <span className="text-sm text-muted-foreground">Сегменты:</span>
              <button onClick={addSegment} className="px-3 py-1 bg-primary text-white rounded text-sm hover:bg-primary/90">
                {mode === "in" ? "Добавить начало" : mode === "out" ? "Добавить конец" : "Добавить сегмент"}
              </button>
              <button onClick={() => { setMode("in"); setSegments([]); }} className="px-3 py-1 border border-border rounded text-sm hover:bg-slate-50">
                Сбросить
              </button>
              <span className="text-xs text-muted-foreground">
                {segments.map((seg, i) => `Сегмент ${i + 1}: ${seg.start.toFixed(1)}s - ${seg.end.toFixed(1)}s`).join(" | ")}
              </span>
            </div>
            <canvas
              ref={canvasRef}
              width={800}
              height={60}
              className="w-full h-[60px] bg-[#010409] border border-border rounded cursor-pointer"
              onClick={handleCanvasClick}
            />
          </div>
          <div className="flex justify-between">
            <button onClick={onBack} className="px-4 py-2 border border-border rounded-lg hover:bg-slate-50 transition-colors">
              ← Назад
            </button>
            <button onClick={handleApply} disabled={segments.length === 0} className="px-6 py-2 bg-primary text-white rounded-lg font-bold hover:bg-primary/90 disabled:opacity-50 disabled:cursor-not-allowed transition-colors">
              Применить обрезку
            </button>
          </div>
        </div>
      </div>
    </div>
  );
}
