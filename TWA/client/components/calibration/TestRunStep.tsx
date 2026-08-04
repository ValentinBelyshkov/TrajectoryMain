import { useState, type RefObject } from "react";
import { Play, AlertCircle, CheckCircle2, Video } from "lucide-react";
import { Button } from "@/components/ui/button";
import { Alert, AlertDescription, AlertTitle } from "@/components/ui/alert";

interface TestRunStepProps {
  projectId: string;
  onSuccess: () => void;
  onBack: () => void;
  hasVideoStream: boolean;
  videoCanvasRef: RefObject<HTMLCanvasElement | null>;
}

export function TestRunStep({ 
  projectId, 
  onSuccess, 
  onBack,
  hasVideoStream,
  videoCanvasRef
}: TestRunStepProps) {
  const [isTestRunning, setIsTestRunning] = useState(false);
  const [testRunError, setTestRunError] = useState<string | null>(null);
  const [isSuccess, setIsSuccess] = useState(false);

  const handleTestRun = async () => {
    setIsTestRunning(true);
    setTestRunError(null);
    try {
      const response = await fetch(`/api/control/terraslam/slam/test-run?project_id=${projectId}`, {
        method: "POST",
      });
      const data = await response.json();
      if (data.success) {
        setIsSuccess(true);
        // Wait a bit to show success state before proceeding
        setTimeout(() => {
          onSuccess();
        }, 1500);
      } else {
        setTestRunError(data.error || "Ошибка при выполнении тест-рана. Файл .osa не найден.");
      }
    } catch (err) {
      setTestRunError("Не удалось связаться с сервером");
    } finally {
      setIsTestRunning(false);
    }
  };

  return (
    <div className="flex flex-col h-full max-w-4xl mx-auto p-6">
      <div className="mb-6">
        <h2 className="text-2xl font-bold text-slate-800">Тестовый запуск SLAM</h2>
        <p className="text-slate-500">
          Проверка инициализации системы перед началом калибровки
        </p>
      </div>

      <div className="flex-1 min-h-[400px] rounded-xl overflow-hidden border-2 border-slate-200 shadow-inner bg-black relative">
        {/* Video Canvas */}
        <canvas
          ref={videoCanvasRef}
          className={`absolute inset-0 w-full h-full object-contain ${hasVideoStream ? 'opacity-100' : 'opacity-0'}`}
          style={{ imageRendering: 'pixelated' }}
        />

        {/* Placeholder when no video */}
        {!hasVideoStream && (
          <div className="absolute inset-0 flex items-center justify-center bg-gray-900">
            <div className="text-center">
              <Video className="w-16 h-16 text-white/30 mx-auto mb-4" />
              <p className="text-white/50 text-sm">Видеопоток с камеры дрона</p>
              <p className="text-white/40 text-xs mt-2">Ожидание подключения...</p>
            </div>
          </div>
        )}
        
        {isSuccess && (
          <div className="absolute inset-0 bg-black/40 backdrop-blur-sm flex items-center justify-center z-20 animate-in fade-in duration-500">
            <div className="bg-white p-8 rounded-2xl shadow-2xl text-center scale-110">
              <CheckCircle2 className="w-16 h-16 text-green-500 mx-auto mb-4" />
              <h3 className="text-xl font-bold text-slate-900">Успешно!</h3>
              <p className="text-slate-500">Инициализация завершена, файл .osa создан</p>
            </div>
          </div>
        )}
      </div>

      <div className="mt-8">
        {testRunError && (
          <Alert variant="destructive" className="mb-6">
            <AlertCircle className="h-4 w-4" />
            <AlertTitle>Ошибка инициализации</AlertTitle>
            <AlertDescription>
              {testRunError}
            </AlertDescription>
          </Alert>
        )}

        <div className="flex flex-col items-center gap-4">
          <Button 
            onClick={handleTestRun} 
            disabled={isTestRunning || isSuccess} 
            size="lg" 
            className="w-full max-w-md h-14 text-lg font-bold gap-3 shadow-lg shadow-primary/20"
          >
            {isTestRunning ? (
              <div className="w-6 h-6 border-3 border-white border-t-transparent rounded-full animate-spin" />
            ) : (
              <Play className="w-6 h-6" />
            )}
            {isTestRunning ? "Выполняется инициализация..." : "Запустить тест (10 сек)"}
          </Button>
          
          <Button 
            variant="ghost" 
            onClick={onBack}
            disabled={isTestRunning || isSuccess}
          >
            Вернуться к инструкции
          </Button>
        </div>
      </div>
    </div>
  );
}
