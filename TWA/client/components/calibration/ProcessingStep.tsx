import { useState } from "react";

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

  return (
    <div className="flex flex-col h-full">
      <div className="flex-1 flex items-center justify-center p-6">
        <div className="max-w-2xl w-full space-y-6">
          <div className="text-center">
            <h3 className="text-2xl font-bold text-foreground mb-2">SLAM обработка видео</h3>
            <p className="text-muted-foreground">
              Извлечение кадров, запуск ORB-SLAM3, сохранение поз
            </p>
          </div>

          {progress && (
            <div className="p-4 bg-slate-50 border border-border rounded-lg">
              <p className="text-sm text-muted-foreground mb-2">Прогресс:</p>
              <p className="font-mono text-sm text-foreground">{progress}</p>
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
            <button onClick={onBack} className="px-6 py-3 border border-border rounded-lg hover:bg-slate-50 transition-colors">
              ← Назад
            </button>
            {onSkip && !isRunning && (
              <button onClick={onSkip} className="px-6 py-3 border border-slate-300 text-slate-600 rounded-lg hover:bg-slate-50 transition-colors">
                Пропустить →
              </button>
            )}
          </div>
        </div>
      </div>
    </div>
  );
}
