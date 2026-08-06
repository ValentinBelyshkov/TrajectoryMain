#!/bin/bash

SERVICE_NAME="terraslam-manager.service"

echo "Останавливаю $SERVICE_NAME..."

if systemctl is-active --quiet "$SERVICE_NAME"; then
    sudo systemctl stop "$SERVICE_NAME"
    
    if systemctl is-active --quiet "$SERVICE_NAME"; then
        echo "Ошибка: не удалось остановить $SERVICE_NAME"
        exit 1
    else
        echo "$SERVICE_NAME успешно остановлена"
    fi
else
    echo "$SERVICE_NAME уже неактивна"
fi
