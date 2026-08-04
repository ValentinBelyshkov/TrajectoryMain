import { Play } from "lucide-react";

interface InstructionsStepProps {
  onNext: () => void;
}

export function InstructionsStep({ onNext }: InstructionsStepProps) {
  return (
    <div className="p-12 max-w-lg mx-auto w-full">
      <div className="text-center mb-8">
        <div className="w-20 h-20 bg-blue-100 rounded-full flex items-center justify-center mx-auto mb-4">
          <span className="text-4xl">🔧</span>
        </div>
        <h2 className="text-2xl font-bold text-foreground mb-2">
          Калибровка системы
        </h2>
        <p className="text-muted-foreground">
          Процесс калибровки состоит из нескольких этапов
        </p>
      </div>

        <div className="space-y-4 mb-8">
          <div className="p-4 bg-blue-50 border border-blue-200 rounded-lg">
            <h3 className="font-semibold text-foreground mb-2">
              Создание калибровочного файла
            </h3>
            <p className="text-sm text-muted-foreground">
              Следуйте инструкциям ниже для создания файла привязки координат
            </p>
          </div>

          <div className="space-y-3">
            <div className="flex gap-3 items-start">
              <div className="flex items-center justify-center w-8 h-8 rounded-full bg-primary text-white font-bold text-sm flex-shrink-0 mt-1">
                1
              </div>
              <div>
                <p className="font-semibold text-foreground">
                  Ознакомьтесь с инструкцией
                </p>
                <p className="text-sm text-muted-foreground">
                  Понять процесс выбора контрольных точек
                </p>
              </div>
            </div>

            <div className="flex gap-3 items-start">
              <div className="flex items-center justify-center w-8 h-8 rounded-full bg-secondary text-white font-bold text-sm flex-shrink-0 mt-1">
                2
              </div>
              <div>
                <p className="font-semibold text-foreground">
                  Загрузите изображение
                </p>
                <p className="text-sm text-muted-foreground">
                  Выберите снимок с дрона для калибровки
                </p>
              </div>
            </div>

            <div className="flex gap-3 items-start">
              <div className="flex items-center justify-center w-8 h-8 rounded-full bg-accent text-white font-bold text-sm flex-shrink-0 mt-1">
                3
              </div>
              <div>
                <p className="font-semibold text-foreground">
                  Установите контрольные точки
                </p>
                <p className="text-sm text-muted-foreground">
                  Установите по 1 точке близко к центру на как минимум 5 изображениях
                </p>
              </div>
            </div>

            <div className="flex gap-3 items-start">
              <div className="flex items-center justify-center w-8 h-8 rounded-full bg-green-500 text-white font-bold text-sm flex-shrink-0 mt-1">
                4
              </div>
              <div>
                <p className="font-semibold text-foreground">
                  Сохраните GCP файл
                </p>
                <p className="text-sm text-muted-foreground">
                  Файл будет сохранён в папку проекта с заголовком +proj=utm
                </p>
              </div>
            </div>
          </div>
        </div>
    </div>
  );
}
