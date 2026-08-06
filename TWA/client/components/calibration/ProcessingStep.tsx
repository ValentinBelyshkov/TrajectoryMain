import { useState } from "react";
import { StepLayout } from "./StepLayout";

interface ProcessingStepProps {
  sessionId: string;
  progress: string;
  onComplete: () => void;
  onBack: () => void;
  onSkip?: () => void;
}

export function ProcessingStep({ sessionId, progress, onComplete, onBack, onSkip }: ProcessingStepProps) {
  const [isRunning, setIsRunning] = useState(false);

  const runProcessing = async () => {
    setIsRunning(true);
    try {
      await onComplete();
    } catch (err) {
      console.error("Processing failed:", err);
    } finally {
      setIsRunning(false);
    }
  };

  // Parse "(done/total)" from the progress string to render a progress bar
  const frameMatch = progress ? progress.match(/\((\d+)\/(\d+)\)/) : null;
  const percent = frameMatch
    ? Math.round((Number(frameMatch[1]) / Number(frameMatch[2])) * 100)
    : null;

  return (
    <StepLayout>
      <div className="text-center">
        <h3 className="text-2xl font-bold text-foreground mb-2">SLAM обработка видео</h3>
        <p className="text-muted-foreground">
          Извлечение кадров, запуск ORB-SLAM3, сохранение поз
        </p>
      </div>

      {progress && (
        <div className="p-4 bg-slate-50 border border-border rounded-lg space-y-3">
          <p className="text-sm text-muted-foreground">Статус обработки:</p>
          <p className="font-mono text-sm text-foreground">{progress}</p>
          {percent !== null && (
            <div className="w-full h-2 bg-slate-200 rounded-full overflow-hidden">
              <div
                className="h-full bg-primary transition-all duration-500"
                style={{ width: `${percent}%` }}
              />
            </div>
          )}
          {isRunning && (
            <p className="text-xs text-muted-foreground">
              Дождитесь завершения обработки. Переход к выбору точек произойдёт автоматически.
            </p>
          )}
        </div>
      )}

      <div className="flex justify-center gap-4">
        <button
          onClick={runProcessing}
          disabled={isRunning}
          className="px-8 py-3 bg-primary text-white rounded-lg font-bold hover:bg-primary/90 disabled:opacity-50 disabled:cursor-not-allowed transition-colors flex items-center gap-2"
        >
          {isRunning ? (
            <>
              <div className="w-5 h-5 border-2 border-white border-t-transparent rounded-full animate-spin" />
              Обработка...
            </>
          ) : (
            "▶ Обработать"
          )}
        </button>
        <button onClick={onBack} disabled={isRunning} className="px-6 py-3 border border-border rounded-lg hover:bg-slate-50 transition-colors disabled:opacity-50 disabled:cursor-not-allowed">
          ← Назад
        </button>
        {onSkip && !isRunning && (
          <button onClick={onSkip} className="px-6 py-3 border border-slate-300 text-slate-600 rounded-lg hover:bg-slate-50 transition-colors">
            Пропустить →
          </button>
        )}
      </div>
    </StepLayout>
  );
}
