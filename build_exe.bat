@echo off
REM =============================================================================
REM  build_exe.bat  -  Compile PGuard Navigator en executable Windows (.exe)
REM =============================================================================
REM  Prerequis : pip install -r requirements.txt pyinstaller
REM  Resultat  : dist\PGuardNavigator\PGuardNavigator.exe
REM =============================================================================

cd /d "%~dp0"

echo [1/3] Verification des dependances...
python -c "import PyQt6; import PyQt6.QtWebEngineWidgets; import folium; import networkx; import route_engine" 2>nul
if errorlevel 1 (
    echo ERREUR : dependances manquantes. Executez : pip install -r requirements.txt
    exit /b 1
)

echo [2/3] Compilation avec PyInstaller...
python -m PyInstaller --noconfirm --clean pguard_navigator.spec
if errorlevel 1 (
    echo ERREUR : la compilation a echoue.
    exit /b 1
)

echo [3/3] Termine !
echo.
echo Executable : %~dp0dist\PGuardNavigator\PGuardNavigator.exe
echo Copiez le dossier entier "dist\PGuardNavigator" pour distribuer l'application.
pause
