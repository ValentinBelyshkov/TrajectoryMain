import { useState } from "react";
import { Button } from "@/components/ui/button";
import { Alert, AlertDescription, AlertTitle } from "@/components/ui/alert";
import { AlertCircle, CheckCircle2, Image, Loader2 } from "lucide-react";
import { cn } from "@/lib/utils";

interface AutoCalibrationImageSelectProps {
  frames: { filename: string; url: string }[];
  onSelect: (filename: string) => void;
  onBack: () => void;
  error?: string | null;
  progress?: "idle" | "downloading" | "matching" | "success" | "error";
  message?: string | null;
}

export function AutoCalibrationImageSelect({
  frames,
  onSelect,
  onBack,
  error,
  progress,
  message,
}: AutoCalibrationImageSelectProps) {
  const [selectedFrame, setSelectedFrame] = useState<string | null>(null);

  const handleConfirm = () => {
    if (selectedFrame) {
      onSelect(selectedFrame);
    }
  };

  return (
    <div className="flex flex-col h-full">
      <div className="bg-gradient-to-r from-blue-600 to-blue-800 text-white p-6 shrink-0">
        <h2 className="text-2xl font-bold mb-2">Выбор изображения</h2>
        <p className="text-blue-100">
          Выберите любой файл из папки frames для сопоставления с картой
        </p>
      </div>

      <div className="flex-1 p-6 overflow-auto">
        {frames.length === 0 ? (
          <div className="flex flex-col items-center justify-center h-full text-muted-foreground">
            <Image className="w-16 h-16 mb-4 opacity-50" />
            <p className="text-lg mb-2">Кадры не найдены</p>
            <p className="text-sm">
              Убедитесь, что в проекте есть загруженные изображения
            </p>
          </div>
        ) : (
          <div className="grid grid-cols-2 md:grid-cols-3 lg:grid-cols-4 xl:grid-cols-5 gap-4">
            {frames.map((frame) => (
              <button
                key={frame.filename}
                onClick={() => setSelectedFrame(frame.filename)}
                className={cn(
                  "relative aspect-video rounded-lg overflow-hidden border-2 transition-all",
                  selectedFrame === frame.filename
                    ? "border-blue-500 ring-2 ring-blue-500/30"
                    : "border-transparent hover:border-slate-300"
                )}
              >
                <img
                  src={frame.url}
                  alt={frame.filename}
                  className="w-full h-full object-cover"
                />
                <div className="absolute bottom-0 left-0 right-0 bg-black/60 text-white text-xs p-1 truncate">
                  {frame.filename}
                </div>
                {selectedFrame === frame.filename && (
                  <div className="absolute top-2 right-2 w-6 h-6 bg-blue-500 rounded-full flex items-center justify-center">
                    <CheckCircle2 className="w-4 h-4 text-white" />
                  </div>
                )}
              </button>
            ))}
          </div>
        )}
      </div>

      {/* Error/Status messages */}
      {error && (
        <Alert variant="destructive" className="m-4 shrink-0">
          <AlertCircle className="h-4 w-4" />
          <AlertTitle>Ошибка калибровки</AlertTitle>
          <AlertDescription>{error}</AlertDescription>
        </Alert>
      )}

      {progress === "matching" && message && (
        <Alert className="m-4 shrink-0 border-blue-200 bg-blue-50">
          <Loader2 className="h-4 w-4 animate-spin" />
          <AlertTitle>Сопоставление изображения</AlertTitle>
          <AlertDescription>{message}</AlertDescription>
        </Alert>
      )}

      {progress === "success" && message && (
        <Alert className="m-4 shrink-0 border-green-200 bg-green-50">
          <CheckCircle2 className="h-4 w-4 text-green-600" />
          <AlertTitle>Калибровка завершена</AlertTitle>
          <AlertDescription>{message}</AlertDescription>
        </Alert>
      )}

      <div className="p-6 border-t bg-slate-50 shrink-0">
        <div className="flex items-center justify-center gap-4">
          <Button variant="outline" onClick={onBack}>
            Назад
          </Button>
          <Button
            onClick={handleConfirm}
            disabled={!selectedFrame || progress === "matching"}
            className="gap-2"
          >
            {progress === "matching" ? (
              <>
                <Loader2 className="w-4 h-4 animate-spin" />
                Сопоставление...
              </>
            ) : (
              <>
                <CheckCircle2 className="w-4 h-4" />
                Сопоставить с картой
              </>
            )}
          </Button>
        </div>
      </div>
    </div>
  );
}
