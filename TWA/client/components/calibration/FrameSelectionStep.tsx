import { useState, useEffect } from "react";
import { Check, Loader2, Image as ImageIcon, ArrowRight } from "lucide-react";
import { Button } from "@/components/ui/button";
import { Card, CardContent } from "@/components/ui/card";
import { ScrollArea } from "@/components/ui/scroll-area";
import { procframe } from "@/lib/api";
import { cn } from "@/lib/utils";

interface Frame {
  filename: string;
  url: string;
}

interface FrameSelectionStepProps {
  projectId: string;
  onFramesSelected: (frames: Frame[]) => void;
  onBack: () => void;
}

export function FrameSelectionStep({
  projectId,
  onFramesSelected,
  onBack,
}: FrameSelectionStepProps) {
  const [frames, setFrames] = useState<Frame[]>([]);
  const [selectedFilenames, setSelectedFilenames] = useState<Set<string>>(new Set());
  const [isLoading, setIsLoading] = useState(true);
  const [error, setError] = useState<string | null>(null);

  useEffect(() => {
    async function loadFrames() {
      try {
        setIsLoading(true);
        const data = await procframe(projectId);
        setFrames(data);
        if (data.length === 0) {
          setError("В папке procframe не найдено изображений");
        }
      } catch (err) {
        setError("Не удалось загрузить список кадров");
        console.error(err);
      } finally {
        setIsLoading(false);
      }
    }

    loadFrames();
  }, [projectId]);

  const toggleFrame = (filename: string) => {
    const newSelected = new Set(selectedFilenames);
    if (newSelected.has(filename)) {
      newSelected.delete(filename);
    } else {
      newSelected.add(filename);
    }
    setSelectedFilenames(newSelected);
  };

  const handleContinue = () => {
    const selected = frames.filter((f) => selectedFilenames.has(f.filename));
    onFramesSelected(selected);
  };

  return (
    <div className="flex flex-col h-full max-w-6xl mx-auto p-6">
      <div className="mb-6 flex justify-between items-end">
        <div>
          <h2 className="text-2xl font-bold text-slate-800">Выбор кадров для калибровки</h2>
          <p className="text-slate-500">
            Выберите изображения из папки procframe для сопоставления с картой
          </p>
        </div>
        <div className="text-sm font-medium text-slate-500 bg-slate-100 px-3 py-1 rounded-full">
          Выбрано: {selectedFilenames.size} (нужно минимум 5)
        </div>
      </div>

      <div className="flex-1 min-h-0 border-2 border-dashed border-slate-200 rounded-xl bg-slate-50/50 overflow-hidden flex flex-col">
        {isLoading ? (
          <div className="flex-1 flex flex-col items-center justify-center">
            <Loader2 className="w-10 h-10 text-primary animate-spin mb-4" />
            <p className="text-slate-500">Загрузка изображений...</p>
          </div>
        ) : error ? (
          <div className="flex-1 flex flex-col items-center justify-center p-6 text-center">
            <div className="w-16 h-16 bg-red-100 rounded-full flex items-center justify-center mb-4">
              <ImageIcon className="w-8 h-8 text-red-500" />
            </div>
            <h3 className="text-lg font-semibold text-slate-800 mb-2">Ошибка</h3>
            <p className="text-slate-500 max-w-md">{error}</p>
            <Button variant="outline" className="mt-4" onClick={() => window.location.reload()}>
              Попробовать снова
            </Button>
          </div>
        ) : (
          <ScrollArea className="flex-1 p-6">
            <div className="grid grid-cols-2 md:grid-cols-3 lg:grid-cols-4 gap-4">
              {frames.map((frame) => {
                const isSelected = selectedFilenames.has(frame.filename);
                return (
                  <Card
                    key={frame.filename}
                    className={cn(
                      "cursor-pointer transition-all duration-200 overflow-hidden border-2 relative group",
                      isSelected
                        ? "border-primary ring-2 ring-primary/20 shadow-md"
                        : "border-transparent hover:border-slate-300"
                    )}
                    onClick={() => toggleFrame(frame.filename)}
                  >
                    <div className="aspect-video relative bg-slate-200">
                      <img
                        src={frame.url}
                        alt={frame.filename}
                        className="w-full h-full object-cover"
                      />
                      <div
                        className={cn(
                          "absolute top-2 right-2 w-6 h-6 rounded-full flex items-center justify-center transition-all duration-200",
                          isSelected
                            ? "bg-primary text-white scale-110"
                            : "bg-black/20 text-white opacity-0 group-hover:opacity-100"
                        )}
                      >
                        <Check className="w-4 h-4" />
                      </div>
                    </div>
                    <CardContent className="p-2 bg-white">
                      <p className="text-xs truncate font-medium text-slate-600" title={frame.filename}>
                        {frame.filename}
                      </p>
                    </CardContent>
                  </Card>
                );
              })}
            </div>
          </ScrollArea>
        )}
      </div>

      <div className="mt-8 flex justify-center gap-4">
        <Button variant="ghost" onClick={onBack} size="lg">
          Назад
        </Button>
        <Button
          onClick={handleContinue}
          disabled={selectedFilenames.size < 5}
          size="lg"
          className="min-w-[200px] gap-2"
        >
          Продолжить
          <ArrowRight className="w-4 h-4" />
        </Button>
      </div>
    </div>
  );
}
