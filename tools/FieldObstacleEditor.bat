@echo off
cd /d "%~dp0"
echo Compiling FieldObstacleEditor...
javac FieldObstacleEditor.java
if errorlevel 1 (
  echo.
  echo Compile failed. Make sure a JDK is on your PATH ^(java -version^).
  pause
  exit /b 1
)
java FieldObstacleEditor
pause
