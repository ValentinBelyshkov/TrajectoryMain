@echo off
powershell -ExecutionPolicy Bypass -File "%~dp0dev-manager.ps1" -Action restart -Target all
