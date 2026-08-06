import { useCallback, useEffect, useRef, useState } from "react";
import { getProjectFrames, trimProjectFrames } from "@/lib/api";
import { Button } from "@/components/ui/button";
import { Slider } from "@/components/ui/slider";
import {
  Play,
  Pause,
  SkipBack,
  SkipForward,
  ChevronFirst,
  ChevronLast,
  Scissors,
  ArrowLeft,
  FastForward,
} from "lucide-react";

interface FrameTrimmingStepProps {
  projectId?: string;
  onApplyTrim: (keepFilenames: string[]) => void;
  onBack: () => void;
  onSkip?: () => void;
}

const SPEEDS = [2, 4, 8, 16];

export function FrameTrimmingStep({
  projectId,
  onApplyTrim,
  onBack,
  onSkip,
}: FrameTrimmingStepProps) {
  const [frames, setFrames] = useState<{ filename: string; url: string }[]>([]);
  const [loading, setLoading] = useState(true);
  const [error, setError] = useState<string | null>(null);

  const [startIdx, setStartIdx] = useState(0);
  const [endIdx, setEndIdx] = useState(0);

  const [busy, setBusy] = useState(false);
  const [playing, setPlaying] = useState(false);
  const [speedIdx, setSpeedIdx] = useState(0);
  const [viewIdx, setViewIdx] = useState(0);

  const stripRef = useRef<HTMLDivElement | null>(null);

  useEffect(() => {
    if (!projectId) return;
    let active = true;
    setLoading(true);
    setError(null);
    getProjectFrames(projectId)
      .then((f) => {
        if (!active) return;
        setFrames(f);
        setStartIdx(0);
        setEndIdx(Math.max(0, f.length - 1));
      })
      .catch((e) => {
        if (active) setError(e instanceof Error ? e.message : "Не удалось загрузить кадры");
      })
      .finally(() => {
        if (active) setLoading(false);
      });
    return () => {
      active = false;
    };
  }, [projectId]);

  const total = frames.length;

  // Lightweight frame player: only the current frame's <img> is in the DOM,
  // so memory stays bounded. No ffmpeg/object-URL building.
  const intervalRef = useRef<ReturnType<typeof setInterval> | null>(null);
  useEffect(() => {
    if (intervalRef.current) {
      clearInterval(intervalRef.current);
      intervalRef.current = null;
    }
    if (!playing || total === 0) return;
    const delay = 1000 / SPEEDS[speedIdx];
    intervalRef.current = setInterval(() => {
      setViewIdx((i) => (i + 1) % total);
    }, delay);
    return () => {
      if (intervalRef.current) {
        clearInterval(intervalRef.current);
        intervalRef.current = null;
      }
    };
  }, [playing, total, speedIdx]);

  const clampStart = Math.min(startIdx, endIdx);
  const clampEnd = Math.max(startIdx, endIdx);
  const keepFilenames = frames.slice(clampStart, clampEnd + 1).map((f) => f.filename);
  const selectedCount = keepFilenames.length;
  const inSelection = viewIdx >= clampStart && viewIdx <= clampEnd;

  const onStartChange = (v: number) => {
    const n = Math.max(0, Math.min(v, total - 1));
    setStartIdx(n);
    if (n > endIdx) setEndIdx(n);
  };
  const onEndChange = (v: number) => {
    const n = Math.max(0, Math.min(v, total - 1));
    setEndIdx(n);
    if (n < startIdx) setStartIdx(n);
  };

  const scrollToIndex = (idx: number) => {
    const strip = stripRef.current;
    if (!strip) return;
    const child = strip.children[idx] as HTMLElement | undefined;
    if (child) child.scrollIntoView({ behavior: "smooth", inline: "center", block: "nearest" });
  };

  const step = (dir: number) => {
    setPlaying(false);
    setViewIdx((i) => {
      const n = i + dir;
      if (n < 0) return total - 1;
      if (n >= total) return 0;
      return n;
    });
  };

  const jumpToEdge = (edge: "start" | "end") => {
    setPlaying(false);
    setViewIdx(edge === "start" ? clampStart : clampEnd);
    scrollToIndex(edge === "start" ? clampStart : clampEnd);
  };

  const setView = (v: number) => {
    const n = Math.max(0, Math.min(v, total - 1));
    setViewIdx(n);
    scrollToIndex(n);
  };

  const cycleSpeed = () => setSpeedIdx((i) => (i + 1) % SPEEDS.length);

  const handleApply = async () => {
    if (!projectId || selectedCount === 0) return;
    setBusy(true);
    setError(null);
    try {
      await trimProjectFrames(projectId, keepFilenames);
      onApplyTrim(keepFilenames);
    } catch (e) {
      setError(e instanceof Error ? e.message : "Ошибка обрезки кадров");
    } finally {
      setBusy(false);
    }
  };

  return (
    <div className="flex flex-col h-full">
      <div className="flex-1 overflow-auto p-6">
        <div className="max-w-5xl mx-auto space-y-6">
          <div>
            <h3 className="text-2xl font-bold text-foreground mb-1">
              Обрезка записанных кадров
            </h3>
            <p className="text-muted-foreground">
              Запись ведётся покадрово с камеры. Выделите диапазон кадров, который
              нужно оставить — лишние кадры будут удалены с сервера.
            </p>
          </div>

          {loading && (
            <div className="text-center text-muted-foreground py-8">
              Загрузка кадров…
            </div>
          )}

          {!loading && error && (
            <div className="bg-red-50 border border-red-200 text-red-700 rounded-lg p-3 text-sm">
              {error}
            </div>
          )}

          {!loading && total === 0 && !error && (
            <div className="text-center text-muted-foreground py-8">
              Кадры не найдены в папке проекта.
            </div>
          )}

          {!loading && total > 0 && (
            <>
              {/* Player */}
              <div className="rounded-xl overflow-hidden border border-border bg-zinc-950 shadow-lg">
                <div className="relative aspect-video bg-black flex items-center justify-center">
                  {total > 0 ? (
                    <img
                      src={frames[viewIdx].url}
                      alt={frames[viewIdx].filename}
                      className="w-full h-full object-contain"
                    />
                  ) : null}

                  {/* Frame indicator badge */}
                  <div className="absolute top-3 left-3 flex items-center gap-2">
                    <span className="rounded-md bg-black/60 backdrop-blur px-2.5 py-1 text-xs font-medium text-white tabular-nums">
                      Кадр {viewIdx + 1} / {total}
                    </span>
                    <span
                      className={
                        "rounded-md px-2.5 py-1 text-xs font-semibold backdrop-blur " +
                        (inSelection
                          ? "bg-emerald-500/80 text-white"
                          : "bg-amber-500/80 text-white")
                      }
                    >
                      {inSelection ? "В выборке" : "Вне выборки"}
                    </span>
                  </div>

                  {playing && (
                    <div className="absolute top-3 right-3 rounded-md bg-black/60 backdrop-blur px-2.5 py-1 text-xs font-medium text-white flex items-center gap-1.5">
                      <span className="size-2 rounded-full bg-red-500 animate-pulse" />
                      {SPEEDS[speedIdx]} к/с
                    </div>
                  )}
                </div>

                {/* Transport controls */}
                <div className="bg-zinc-900/95 border-t border-zinc-800 px-4 py-3 space-y-3">
                  {/* Seek bar */}
                  <Slider
                    value={[viewIdx]}
                    min={0}
                    max={Math.max(0, total - 1)}
                    step={1}
                    onValueChange={(v) => setView(v[0])}
                    className="[&_[data-orientation=horizontal]]:h-1.5"
                    aria-label="Перемотка кадров"
                  />

                  <div className="flex items-center justify-between gap-2 flex-wrap">
                    <div className="flex items-center gap-1.5">
                      <Button
                        variant="ghost"
                        size="icon"
                        onClick={() => jumpToEdge("start")}
                        disabled={total === 0}
                        className="text-zinc-200 hover:bg-zinc-800 hover:text-white"
                        title="В начало выборки"
                      >
                        <ChevronFirst />
                      </Button>
                      <Button
                        variant="ghost"
                        size="icon"
                        onClick={() => step(-1)}
                        disabled={total === 0}
                        className="text-zinc-200 hover:bg-zinc-800 hover:text-white"
                        title="Предыдущий кадр"
                      >
                        <SkipBack />
                      </Button>
                      <Button
                        variant="secondary"
                        size="icon"
                        onClick={() => setPlaying((p) => !p)}
                        disabled={total === 0}
                        className="size-11 rounded-full bg-white text-zinc-900 hover:bg-zinc-200 shadow"
                        title={playing ? "Пауза" : "Воспроизвести"}
                      >
                        {playing ? <Pause className="size-5" /> : <Play className="size-5 translate-x-0.5" />}
                      </Button>
                      <Button
                        variant="ghost"
                        size="icon"
                        onClick={() => step(1)}
                        disabled={total === 0}
                        className="text-zinc-200 hover:bg-zinc-800 hover:text-white"
                        title="Следующий кадр"
                      >
                        <SkipForward />
                      </Button>
                      <Button
                        variant="ghost"
                        size="icon"
                        onClick={() => jumpToEdge("end")}
                        disabled={total === 0}
                        className="text-zinc-200 hover:bg-zinc-800 hover:text-white"
                        title="В конец выборки"
                      >
                        <ChevronLast />
                      </Button>
                    </div>

                    <Button
                      variant="outline"
                      size="sm"
                      onClick={cycleSpeed}
                      disabled={total === 0}
                      className="gap-1.5 border-zinc-700 bg-zinc-800/60 text-zinc-100 hover:bg-zinc-700"
                      title="Скорость воспроизведения"
                    >
                      <FastForward className="size-4" />
                      {SPEEDS[speedIdx]} к/с
                    </Button>
                  </div>
                </div>
              </div>

              {/* Selection info */}
              <div className="flex items-center justify-between rounded-lg border border-border bg-muted/50 px-4 py-3">
                <span className="text-sm text-muted-foreground">
                  Выбрано кадров:{" "}
                  <b className="text-foreground">{selectedCount}</b> из {total}
                </span>
                <span className="text-sm text-muted-foreground">
                  Диапазон: с{" "}
                  <b className="text-foreground">{clampStart + 1}</b>-го по{" "}
                  <b className="text-foreground">{clampEnd + 1}</b>-й
                </span>
              </div>

              {/* Thumbnail strip */}
              <div>
                <p className="text-sm font-medium text-muted-foreground mb-2">
                  Лента кадров — кликните по краям, чтобы задать границы выборки
                </p>
                <div
                  ref={stripRef}
                  className="flex gap-1.5 overflow-x-auto p-2.5 rounded-xl border border-border bg-muted/40"
                  style={{ maxHeight: 168 }}
                >
                  {frames.map((f, i) => {
                    const selected = i >= clampStart && i <= clampEnd;
                    const isCurrent = i === viewIdx;
                    return (
                      <img
                        key={f.filename}
                        src={f.url}
                        loading="lazy"
                        onClick={() => {
                          if (i < clampStart) onStartChange(i);
                          else if (i > clampEnd) onEndChange(i);
                          else setView(i);
                        }}
                        className="h-28 w-auto object-cover rounded-md cursor-pointer shrink-0 transition-all hover:scale-[1.02]"
                        style={{
                          opacity: selected ? 1 : 0.35,
                          outline: isCurrent
                            ? "3px solid #fff"
                            : selected
                            ? "2px solid #22c55e"
                            : "2px solid transparent",
                          boxShadow: isCurrent ? "0 0 0 2px #22c55e" : "none",
                        }}
                        alt={f.filename}
                      />
                    );
                  })}
                </div>
              </div>

              {/* Range sliders */}
              <div className="grid grid-cols-1 sm:grid-cols-2 gap-5">
                <div className="space-y-2">
                  <div className="flex items-center justify-between text-sm">
                    <span className="font-medium text-muted-foreground">Начало</span>
                    <span className="tabular-nums text-foreground">кадр {clampStart + 1}</span>
                  </div>
                  <Slider
                    value={[startIdx]}
                    min={0}
                    max={Math.max(0, total - 1)}
                    step={1}
                    onValueChange={(v) => onStartChange(v[0])}
                    onValueCommit={() => scrollToIndex(startIdx)}
                    className="[&_[data-orientation=horizontal]]:h-2"
                  />
                </div>
                <div className="space-y-2">
                  <div className="flex items-center justify-between text-sm">
                    <span className="font-medium text-muted-foreground">Конец</span>
                    <span className="tabular-nums text-foreground">кадр {clampEnd + 1}</span>
                  </div>
                  <Slider
                    value={[endIdx]}
                    min={0}
                    max={Math.max(0, total - 1)}
                    step={1}
                    onValueChange={(v) => onEndChange(v[0])}
                    onValueCommit={() => scrollToIndex(endIdx)}
                    className="[&_[data-orientation=horizontal]]:h-2"
                  />
                </div>
              </div>
            </>
          )}
        </div>
      </div>

      {/* Footer actions */}
      <div className="p-4 border-t bg-muted/40 flex justify-between shrink-0">
        <Button variant="outline" onClick={onBack} className="gap-1.5">
          <ArrowLeft className="size-4" />
          Назад
        </Button>
        <div className="flex gap-2">
          {onSkip && (
            <Button
              variant="ghost"
              onClick={onSkip}
              className="gap-1.5"
            >
              Пропустить →
            </Button>
          )}
          <Button
            onClick={handleApply}
            disabled={busy || loading || selectedCount === 0}
            className="gap-1.5"
          >
            <Scissors className="size-4" />
            {busy ? "Применение…" : "Применить обрезку"}
          </Button>
        </div>
      </div>
    </div>
  );
}
