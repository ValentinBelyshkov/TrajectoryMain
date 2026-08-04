import { useState } from "react";

interface FinalizeStepProps {
  sessionId: string;
  transform: any;
  onComplete: () => void;
  onBack: () => void;
}

export function FinalizeStep({ sessionId, transform, onComplete, onBack }: FinalizeStepProps) {
  const [isGenerating, setIsGenerating] = useState(false);
  const [result, setResult] = useState<any>(null);
  const [error, setError] = useState<string | null>(null);

  const generateCalibration = async () => {
    setIsGenerating(true);
    setError(null);
    try {
      const res = await fetch(`/api/v1/calibration/session/${sessionId}/finalize`, {
        method: "POST",
      });
      const data = await res.json();
      if (!data.success) throw new Error(data.detail || "Finalize failed");
      setResult(data);
      onComplete();
    } catch (err) {
      setError(err instanceof Error ? err.message : "Ошибка создания calib.gpc");
    } finally {
      setIsGenerating(false);
    }
  };

  return (
    <div className="flex flex-col h-full">
      <div className="flex-1 flex items-center justify-center p-6">
        <div className="max-w-2xl w-full space-y-6">
          <div className="text-center">
            <h3 className="text-2xl font-bold text-foreground mb-2">Создание калибровочного файла</h3>
            <p className="text-muted-foreground">
              Генерация calib.gpc на основе посчитанного преобразования
            </p>
          </div>

          {transform && (
            <div className="p-4 bg-slate-50 border border-border rounded-lg">
              <h4 className="font-bold mb-2">Преобразование:</h4>
              <pre className="text-xs font-mono text-foreground overflow-auto">
                {JSON.stringify(transform, null, 2)}
              </pre>
            </div>
          )}

          {error && (
            <div className="p-4 bg-red-50 border border-red-200 rounded-lg text-red-700">
              {error}
            </div>
          )}

          {result && (
            <div className="p-4 bg-green-50 border border-green-200 rounded-lg text-green-700">
              <h4 className="font-bold mb-2">✅ calib.gpc создан</h4>
              <pre className="text-xs font-mono overflow-auto">{result.content}</pre>
            </div>
          )}

          <div className="flex justify-center gap-4">
            <button
              onClick={generateCalibration}
              disabled={isGenerating || !transform}
              className="px-8 py-3 bg-green-600 text-white rounded-lg font-bold hover:bg-green-700 disabled:opacity-50 disabled:cursor-not-allowed transition-colors flex items-center gap-2"
            >
              {isGenerating ? (
                <>
                  <div className="w-5 h-5 border-2 border-white border-t-transparent rounded-full animate-spin" />
                  Создание...
                </>
              ) : (
                "💾 Создать calib.gpc"
              )}
            </button>
            <button onClick={onBack} className="px-6 py-3 border border-border rounded-lg hover:bg-slate-50 transition-colors">
              ← Назад
            </button>
          </div>
        </div>
      </div>
    </div>
  );
}
