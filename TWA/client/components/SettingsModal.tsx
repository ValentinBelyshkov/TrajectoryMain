import { useState, useEffect } from "react";
import {
  Dialog,
  DialogContent,
  DialogHeader,
  DialogTitle,
  DialogFooter,
} from "@/components/ui/dialog";
import { Button } from "@/components/ui/button";
import { Input } from "@/components/ui/input";
import { Label } from "@/components/ui/label";
import {
  Select,
  SelectContent,
  SelectItem,
  SelectTrigger,
  SelectValue,
} from "@/components/ui/select";
import { Tabs, TabsContent, TabsList, TabsTrigger } from "@/components/ui/tabs";
import { AppSettings, ProtocolType, TransmitterType, SessionType } from "@shared/api";
import { getAppSettings, saveAppSettings } from "@/lib/api";
import { toast } from "sonner";
import { Settings2, Cpu, Cable, Activity } from "lucide-react";

interface SettingsModalProps {
  open: boolean;
  onOpenChange: (open: boolean) => void;
}

export function SettingsModal({ open, onOpenChange }: SettingsModalProps) {
  const [settings, setSettings] = useState<AppSettings | null>(null);
  const [isLoading, setIsLoading] = useState(false);

  useEffect(() => {
    if (open) {
      loadSettings();
    }
  }, [open]);

  const loadSettings = async () => {
    try {
      const data = await getAppSettings();
      setSettings(data);
    } catch (error) {
      console.error("Failed to load settings:", error);
      toast.error("Ошибка при загрузке настроек");
    }
  };

  const handleSave = async () => {
    if (!settings) return;
    setIsLoading(true);
    try {
      await saveAppSettings(settings);
      toast.success("Настройки успешно сохранены");
      onOpenChange(false);
    } catch (error) {
      console.error("Failed to save settings:", error);
      toast.error("Ошибка при сохранении настроек");
    } finally {
      setIsLoading(false);
    }
  };

  if (!settings) return null;

  return (
    <Dialog open={open} onOpenChange={onOpenChange}>
      <DialogContent className="max-w-2xl max-h-[90vh] overflow-y-auto">
        <DialogHeader>
          <DialogTitle className="flex items-center gap-2 text-xl">
            <Settings2 className="w-6 h-6 text-primary" />
            Настройки системы (Jetson Orin)
          </DialogTitle>
        </DialogHeader>

        <Tabs defaultValue="general" className="mt-4">
          <TabsList className="grid w-full grid-cols-3">
            <TabsTrigger value="general" className="flex gap-2">
              <Cpu className="w-4 h-4" /> Основные
            </TabsTrigger>
            <TabsTrigger value="orb" className="flex gap-2">
              <Activity className="w-4 h-4" /> ORB Extractor
            </TabsTrigger>
            <TabsTrigger value="protocols" className="flex gap-2">
              <Cable className="w-4 h-4" /> Протоколы
            </TabsTrigger>
          </TabsList>

          <TabsContent value="general" className="space-y-6 py-6">
            <div className="grid grid-cols-1 md:grid-cols-2 gap-6">
              <div className="space-y-3">
                <Label className="text-base">Активный протокол</Label>
                <Select
                  value={settings.protocol}
                  onValueChange={(value: ProtocolType) =>
                    setSettings({ ...settings, protocol: value })
                  }
                >
                  <SelectTrigger className="w-full">
                    <SelectValue placeholder="Выберите протокол" />
                  </SelectTrigger>
                  <SelectContent>
                    <SelectItem value="Mavlink">Mavlink</SelectItem>
                    <SelectItem value="MSP">MSP</SelectItem>
                    <SelectItem value="Ublox">Ublox</SelectItem>
                    <SelectItem value="Custom">Custom</SelectItem>
                  </SelectContent>
                </Select>
                <p className="text-xs text-muted-foreground">
                  Основной протокол обмена данными с полетным контроллером
                </p>
              </div>

              <div className="space-y-3">
                <Label className="text-base">Передающее устройство</Label>
                <Select
                  value={settings.transmitter}
                  onValueChange={(value: TransmitterType) =>
                    setSettings({ ...settings, transmitter: value })
                  }
                >
                  <SelectTrigger className="w-full">
                    <SelectValue placeholder="Выберите тип" />
                  </SelectTrigger>
                  <SelectContent>
                    <SelectItem value="UART">UART</SelectItem>
                    <SelectItem value="I2C">I2C</SelectItem>
                    <SelectItem value="USB">USB</SelectItem>
                  </SelectContent>
                </Select>
                <p className="text-xs text-muted-foreground">
                  Физический интерфейс подключения
                </p>
              </div>

              <div className="space-y-3">
                <Label className="text-base">Тип сессии SLAM</Label>
                <Select
                  value={settings.session}
                  onValueChange={(value: SessionType) =>
                    setSettings({ ...settings, session: value })
                  }
                >
                  <SelectTrigger className="w-full">
                    <SelectValue placeholder="Выберите тип сессии" />
                  </SelectTrigger>
                  <SelectContent>
                    <SelectItem value="Mono">Mono</SelectItem>
                    <SelectItem value="Stereo">Stereo</SelectItem>
                    <SelectItem value="Mono+IMU">Mono+IMU</SelectItem>
                    <SelectItem value="Stereo+IMU">Stereo+IMU</SelectItem>
                  </SelectContent>
                </Select>
                <p className="text-xs text-muted-foreground">
                  Конфигурация датчиков для построения карты
                </p>
              </div>
            </div>
          </TabsContent>

          <TabsContent value="orb" className="space-y-6 py-6">
            <div className="bg-slate-50 p-4 rounded-lg mb-4 border border-slate-200">
              <h4 className="font-semibold text-sm mb-1 text-slate-800">Параметры детектора ORB</h4>
              <p className="text-xs text-slate-500">
                Настройки извлечения ключевых точек для визуальной одометрии
              </p>
            </div>
            <div className="grid grid-cols-1 md:grid-cols-2 gap-6">
              <div className="space-y-2">
                <Label>nFeatures (Кол-во точек)</Label>
                <Input
                  type="number"
                  value={settings.orbExtractor.nFeatures}
                  onChange={(e) =>
                    setSettings({
                      ...settings,
                      orbExtractor: {
                        ...settings.orbExtractor,
                        nFeatures: parseInt(e.target.value) || 0,
                      },
                    })
                  }
                />
              </div>
              <div className="space-y-2">
                <Label>scaleFactor (Множитель шкалы)</Label>
                <Input
                  type="number"
                  step="0.1"
                  value={settings.orbExtractor.scaleFactor}
                  onChange={(e) =>
                    setSettings({
                      ...settings,
                      orbExtractor: {
                        ...settings.orbExtractor,
                        scaleFactor: parseFloat(e.target.value) || 0,
                      },
                    })
                  }
                />
              </div>
              <div className="space-y-2">
                <Label>nLevels (Кол-во уровней)</Label>
                <Input
                  type="number"
                  value={settings.orbExtractor.nLevels}
                  onChange={(e) =>
                    setSettings({
                      ...settings,
                      orbExtractor: {
                        ...settings.orbExtractor,
                        nLevels: parseInt(e.target.value) || 0,
                      },
                    })
                  }
                />
              </div>
              <div className="space-y-2">
                <Label>iniThFAST (Порог FAST нач.)</Label>
                <Input
                  type="number"
                  value={settings.orbExtractor.iniThFAST}
                  onChange={(e) =>
                    setSettings({
                      ...settings,
                      orbExtractor: {
                        ...settings.orbExtractor,
                        iniThFAST: parseInt(e.target.value) || 0,
                      },
                    })
                  }
                />
              </div>
              <div className="space-y-2">
                <Label>minThFAST (Порог FAST мин.)</Label>
                <Input
                  type="number"
                  value={settings.orbExtractor.minThFAST}
                  onChange={(e) =>
                    setSettings({
                      ...settings,
                      orbExtractor: {
                        ...settings.orbExtractor,
                        minThFAST: parseInt(e.target.value) || 0,
                      },
                    })
                  }
                />
              </div>
            </div>
          </TabsContent>

          <TabsContent value="protocols" className="space-y-4 py-6">
             <div className="space-y-6">
                {Object.entries(settings.protocolConfigs).map(([key, config]) => (
                  <div key={key} className="border border-slate-200 p-5 rounded-xl bg-white shadow-sm space-y-4">
                    <div className="flex items-center justify-between border-b pb-2 mb-2">
                        <h4 className="font-bold text-lg text-primary">{key}</h4>
                        {settings.protocol === key && (
                            <span className="bg-green-100 text-green-700 text-xs px-2 py-1 rounded-full font-medium">Активен</span>
                        )}
                    </div>
                    <div className="grid grid-cols-1 md:grid-cols-2 gap-4">
                      <div className="space-y-2">
                        <Label className="text-sm font-medium">Скорость передачи (Baud Rate)</Label>
                        <Select 
                          value={config.baudRate?.toString() || "57600"} 
                          onValueChange={(value) => {
                             const newConfigs = {...settings.protocolConfigs};
                             newConfigs[key as ProtocolType] = {...config, baudRate: parseInt(value)};
                             setSettings({...settings, protocolConfigs: newConfigs});
                          }}
                        >
                            <SelectTrigger>
                                <SelectValue placeholder="Выберите скорость" />
                            </SelectTrigger>
                            <SelectContent>
                                <SelectItem value="9600">9600</SelectItem>
                                <SelectItem value="19200">19200</SelectItem>
                                <SelectItem value="38400">38400</SelectItem>
                                <SelectItem value="57600">57600</SelectItem>
                                <SelectItem value="115200">115200</SelectItem>
                                <SelectItem value="230400">230400</SelectItem>
                                <SelectItem value="460800">460800</SelectItem>
                                <SelectItem value="921600">921600</SelectItem>
                            </SelectContent>
                        </Select>
                      </div>
                      <div className="space-y-2">
                        <Label className="text-sm font-medium">Порт устройства</Label>
                        <Input 
                          placeholder="/dev/ttyTHS0"
                          value={config.port || ""} 
                          onChange={(e) => {
                             const newConfigs = {...settings.protocolConfigs};
                             newConfigs[key as ProtocolType] = {...config, port: e.target.value};
                             setSettings({...settings, protocolConfigs: newConfigs});
                          }}
                        />
                      </div>
                    </div>
                  </div>
                ))}
             </div>
          </TabsContent>
        </Tabs>

        <DialogFooter className="mt-8 border-t pt-6">
          <Button variant="ghost" onClick={() => onOpenChange(false)}>
            Отмена
          </Button>
          <Button onClick={handleSave} disabled={isLoading} className="min-w-[120px]">
            {isLoading ? "Сохранение..." : "Сохранить изменения"}
          </Button>
        </DialogFooter>
      </DialogContent>
    </Dialog>
  );
}
