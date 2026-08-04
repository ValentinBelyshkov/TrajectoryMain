import { useState, useCallback } from "react";
import { ChevronLeft, ChevronRight, Check, X, Upload, Play, AlertCircle } from "lucide-react";
import { Button } from "./ui/button";
import { cn } from "@/lib/utils";
import { VideoFeed } from "./VideoFeed";
import { Alert, AlertDescription, AlertTitle } from "./ui/alert";

interface Point {
  x: number;
  y: number;
}

interface FrameData {
  imageUrl: string;
  points: Point[];
}

interface CalibrationWizardProps {
  projectId: string;
  frameUrls?: string[];
  onComplete?: (frames: FrameData[]) => void;
  onCancel?: () => void;
}

export function CalibrationWizard({
  projectId,
  frameUrls = [],
  onComplete,
  onCancel,
}: CalibrationWizardProps) {
  const [currentFrame, setCurrentFrame] = useState(0);
  const [step, setStep] = useState<"instructions" | "test-run" | "recording" | "calibration">(
    frameUrls.length > 0 ? "calibration" : "instructions",
  );
  const [isTestRunning, setIsTestRunning] = useState(false);
  const [testRunError, setTestRunError] = useState<string | null>(null);
  const [frames, setFrames] = useState<FrameData[]>(
    frameUrls.length >= 5
      ? frameUrls.map((url) => ({ imageUrl: url, points: [] }))
      : Array.from({ length: 5 }).map((_, i) => ({ imageUrl: `/placeholder-frame-${i+1}.jpg`, points: [] })),
  );
  const [isCapturing, setIsCapturing] = useState(false);
  const [captureProgress, setCaptureProgress] = useState(0);

  const currentFrameData = frames[currentFrame];
  const totalPointsRequired = 1;
  const isCurrentFrameComplete =
    currentFrameData.points.length === totalPointsRequired;
  const allFramesComplete = frames.every(
    (f) => f.points.length === totalPointsRequired,
  );

  const handleImageClick = useCallback(
    (e: React.MouseEvent<HTMLDivElement>) => {
      if (isCurrentFrameComplete) return;

      const rect = e.currentTarget.getBoundingClientRect();
      const x = ((e.clientX - rect.left) / rect.width) * 100;
      const y = ((e.clientY - rect.top) / rect.height) * 100;

      setFrames((prev) => {
        const newFrames = [...prev];
        newFrames[currentFrame] = {
          ...newFrames[currentFrame],
          points: [...newFrames[currentFrame].points, { x, y }],
        };
        return newFrames;
      });
    },
    [currentFrame, isCurrentFrameComplete],
  );

  const handleStartCapture = async () => {
    setStep("recording");
    setIsCapturing(true);
    setCaptureProgress(0);

    // Simulate 15-second recording
    for (let i = 0; i <= 15; i++) {
      await new Promise((resolve) => setTimeout(resolve, 1000));
      setCaptureProgress((i / 15) * 100);
    }

    // In production, this would call /api/calibration/start
    // For now, simulate getting frame URLs
    setIsCapturing(false);
    setStep("calibration");
  };

  const handleTestRun = async () => {
    setIsTestRunning(true);
    setTestRunError(null);
    try {
      const response = await fetch(`/api/terraslam/slam/test-run?project_id=${projectId}`, {
        method: "POST",
      });
      const data = await response.json();
      if (data.success) {
        handleStartCapture();
      } else {
        setTestRunError(data.error || "Ошибка при выполнении тест-рана. Файл .osa не найден.");
      }
    } catch (err) {
      setTestRunError("Не удалось связаться с сервером");
    } finally {
      setIsTestRunning(false);
    }
  };

  const handleResetFrame = (frameIndex: number) => {
    setFrames((prev) => {
      const newFrames = [...prev];
      newFrames[frameIndex] = { ...newFrames[frameIndex], points: [] };
      return newFrames;
    });
  };

  const handleSave = () => {
    onComplete?.(frames);
  };

  return (
    <div className="fixed inset-0 bg-black/80 flex items-center justify-center z-50">
      <div className="bg-white w-full h-full overflow-hidden flex flex-col">
        {/* Header */}
        <div className="bg-gradient-to-r from-blue-600 to-blue-800 text-white p-6">
          <h2 className="text-2xl font-bold mb-2">Калибровка камеры</h2>
          <p className="text-blue-100">
            {step === "calibration" 
              ? "Установите по 1 контрольной точке на каждом из 5 кадров"
              : step === "test-run"
              ? "Тестовый запуск и инициализация"
              : step === "recording"
              ? "Запись видео для калибровки"
              : "Инструкция по калибровке"}
          </p>
        </div>

        {/* Progress bar (only show during calibration) */}
        {step === "calibration" && (
          <div className="bg-gray-100 px-6 py-3">
            <div className="flex items-center gap-2">
              {frames.map((frame, idx) => (
                <div key={idx} className="flex-1">
                  <div
                    className={cn(
                      "h-2 rounded-full transition-colors",
                      frame.points.length === totalPointsRequired
                        ? "bg-green-500"
                        : idx === currentFrame
                          ? "bg-blue-500"
                          : "bg-gray-300",
                    )}
                    style={{ width: "100%" }}
                  />
                  <p className="text-xs text-center mt-1 text-muted-foreground">
                    Кадр {idx + 1} ({frame.points.length}/{totalPointsRequired})
                  </p>
                </div>
              ))}
            </div>
          </div>
        )}

        {/* Main content */}
        <div className="flex-1 p-6 overflow-auto">
          {step === "instructions" ? (
            <div className="text-center py-12">
              <div className="w-24 h-24 bg-blue-100 rounded-full flex items-center justify-center mx-auto mb-6">
                <Upload className="w-12 h-12 text-blue-600" />
              </div>
              <h3 className="text-xl font-bold text-foreground mb-3">
                Начало калибровки
              </h3>
              <p className="text-muted-foreground mb-6 max-w-md mx-auto">
                Поднимите дрон на высоту ~1.5 метра. Перед началом калибровки необходимо
                выполнить тестовый запуск SLAM для проверки инициализации.
              </p>
              <Button onClick={() => setStep("test-run")} size="lg" className="gap-2">
                Продолжить
                <ChevronRight className="w-5 h-5" />
              </Button>
            </div>
          ) : step === "test-run" ? (
            <div className="flex flex-col h-full max-w-4xl mx-auto">
              <div className="flex-1 min-h-[300px] rounded-lg overflow-hidden border border-border shadow-md bg-black">
                <VideoFeed projectId={projectId} />
              </div>
              <div className="mt-6 text-center">
                <h3 className="text-xl font-bold text-foreground mb-2">
                  Тестовый запуск SLAM
                </h3>
                <p className="text-muted-foreground mb-6">
                  Нажмите кнопку "Старт" для проверки инициализации SLAM.
                  Процесс займет около 10 секунд.
                </p>
                
                {testRunError && (
                  <Alert variant="destructive" className="mb-6 text-left">
                    <AlertCircle className="h-4 w-4" />
                    <AlertTitle>Ошибка инициализации</AlertTitle>
                    <AlertDescription>
                      {testRunError}
                    </AlertDescription>
                  </Alert>
                )}

                <div className="flex flex-col items-center gap-3">
                  <Button 
                    onClick={handleTestRun} 
                    disabled={isTestRunning} 
                    size="lg" 
                    className="gap-2 px-8"
                  >
                    {isTestRunning ? (
                      <div className="w-5 h-5 border-2 border-white border-t-transparent rounded-full animate-spin" />
                    ) : (
                      <Play className="w-5 h-5" />
                    )}
                    {isTestRunning ? "Инициализация..." : "Старт"}
                  </Button>
                  
                  {testRunError && (
                    <Button 
                      variant="ghost" 
                      onClick={() => setStep("instructions")}
                    >
                      Вернуться к инструкции
                    </Button>
                  )}
                </div>
              </div>
            </div>
          ) : step === "recording" ? (
            <div className="text-center py-12">
              <div className="w-32 h-32 border-4 border-blue-500 border-t-transparent rounded-full mx-auto mb-6 animate-spin" />
              <h3 className="text-xl font-bold text-foreground mb-3">
                Идёт запись...
              </h3>
              <p className="text-muted-foreground mb-4">
                Пожалуйста, держите дрон стабильно
              </p>
              <div className="max-w-xs mx-auto bg-gray-200 rounded-full h-4 overflow-hidden">
                <div
                  className="bg-blue-500 h-full transition-all duration-300"
                  style={{ width: `${captureProgress}%` }}
                />
              </div>
              <p className="text-sm text-muted-foreground mt-2">
                {Math.round(captureProgress)}% / 100%
              </p>
            </div>
          ) : (
            <div className="grid grid-cols-1 md:grid-cols-2 gap-6">
              {/* Image area */}
              <div>
                <h3 className="text-lg font-semibold text-foreground mb-3">
                  Кадр {currentFrame + 1} из 3
                </h3>
                <div
                  onClick={handleImageClick}
                  className={cn(
                    "relative aspect-video bg-gray-100 rounded-lg overflow-hidden cursor-crosshair",
                    isCurrentFrameComplete
                      ? "cursor-default"
                      : "hover:ring-2 hover:ring-blue-500",
                  )}
                >
                  <img
                    src={currentFrameData.imageUrl}
                    alt={`Кадр ${currentFrame + 1}`}
                    className="w-full h-full object-contain"
                  />

                  {/* Points overlay */}
                  {currentFrameData.points.map((point, idx) => (
                    <div
                      key={idx}
                      className="absolute w-8 h-8 -translate-x-1/2 -translate-y-1/2 rounded-full border-2 border-white shadow-lg flex items-center justify-center"
                      style={{
                        left: `${point.x}%`,
                        top: `${point.y}%`,
                        backgroundColor:
                          idx === 0
                            ? "#ef4444"
                            : idx === 1
                              ? "#f97316"
                              : idx === 2
                                ? "#eab308"
                                : idx === 3
                                  ? "#22c55e"
                                  : "#3b82f6",
                      }}
                    >
                      <span className="text-white text-xs font-bold">
                        {idx + 1}
                      </span>
                    </div>
                  ))}

                  {/* Instructions */}
                  {!isCurrentFrameComplete && (
                    <div className="absolute inset-0 bg-black/30 flex items-center justify-center">
                      <p className="text-white font-medium px-4 py-2 bg-black/60 rounded-lg">
                        Кликните для установки точки{" "}
                        {currentFrameData.points.length + 1}
                      </p>
                    </div>
                  )}

                  {isCurrentFrameComplete && (
                    <div className="absolute inset-0 bg-green-500/20 flex items-center justify-center">
                      <div className="bg-green-600 text-white px-4 py-2 rounded-lg font-medium flex items-center gap-2">
                        <Check className="w-4 h-4" />
                        Все точки установлены
                      </div>
                    </div>
                  )}
                </div>

                {/* Reset button */}
                {currentFrameData.points.length > 0 && (
                  <Button
                    variant="outline"
                    size="sm"
                    onClick={() => handleResetFrame(currentFrame)}
                    className="mt-3"
                  >
                    <X className="w-4 h-4 mr-2" />
                    Сбросить точки кадра
                  </Button>
                )}
              </div>

              {/* Points list */}
              <div>
                <h3 className="text-lg font-semibold text-foreground mb-3">
                  Установленные точки
                </h3>
                <div className="space-y-2">
                  {currentFrameData.points.map((point, idx) => (
                    <div
                      key={idx}
                      className="flex items-center justify-between p-3 bg-gray-50 rounded-lg"
                    >
                      <div className="flex items-center gap-3">
                        <div
                          className="w-6 h-6 rounded-full flex items-center justify-center text-white text-xs font-bold"
                          style={{
                            backgroundColor:
                              idx === 0
                                ? "#ef4444"
                                : idx === 1
                                  ? "#f97316"
                                  : idx === 2
                                    ? "#eab308"
                                    : idx === 3
                                      ? "#22c55e"
                                      : "#3b82f6",
                          }}
                        >
                          {idx + 1}
                        </div>
                        <span className="font-mono text-sm">
                          ({point.x.toFixed(1)}%, {point.y.toFixed(1)}%)
                        </span>
                      </div>
                    </div>
                  ))}

                  {currentFrameData.points.length === 0 && (
                    <p className="text-muted-foreground text-center py-8">
                      Точки не установлены
                    </p>
                  )}
                </div>
              </div>
            </div>
          )}
        </div>

        {/* Footer */}
        <div className="bg-gray-50 border-t border-border p-6 flex flex-col items-center gap-4">
          <div className="flex items-center justify-center gap-3 w-full max-w-md">
            {step === "calibration" && (
              <>
                <Button
                  variant="outline"
                  onClick={() => setCurrentFrame((prev) => Math.max(0, prev - 1))}
                  disabled={currentFrame === 0}
                  className="flex-1"
                >
                  <ChevronLeft className="w-4 h-4 mr-1" />
                  Назад
                </Button>

                {currentFrame < 4 ? (
                  <Button
                    onClick={() => setCurrentFrame((prev) => prev + 1)}
                    disabled={!isCurrentFrameComplete}
                    className="flex-1"
                  >
                    Далее
                    <ChevronRight className="w-4 h-4 ml-1" />
                  </Button>
                ) : (
                  <Button
                    onClick={handleSave}
                    disabled={!allFramesComplete}
                    className="gap-2 flex-1"
                  >
                    <Check className="w-4 h-4" />
                    Сохранить калибровку
                  </Button>
                )}
              </>
            )}
            
            {step !== "calibration" && (
              <Button variant="ghost" onClick={onCancel} className="text-muted-foreground">
                Отмена калибровки
              </Button>
            )}
          </div>
        </div>
      </div>
    </div>
  );
}
