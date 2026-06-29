param(
    [string]$PortName = "COM5",
    [int]$BaudRate = 115200,
    [string]$TestName = "pololu_bump_encoder_maze"
)

$LogDir = Join-Path $env:USERPROFILE\Desktop "cps"

if (!(Test-Path $LogDir)) {
    New-Item -ItemType Directory -Path $LogDir | Out-Null
}

$AvailablePorts = [System.IO.Ports.SerialPort]::GetPortNames()

if ($AvailablePorts.Count -eq 0) {
    Write-Host "No serial ports found."
    Write-Host "Press RESET only on the robot, then run the script again."
    exit 1
}

if ($AvailablePorts -notcontains $PortName) {
    Write-Host "Requested port $PortName was not found."
    Write-Host "Available ports:"
    $AvailablePorts | ForEach-Object { Write-Host "  $_" }
    exit 1
}

$SafeTestName = $TestName -replace '[^a-zA-Z0-9_-]', '_'
$Timestamp = Get-Date -Format "yyyyMMdd_HHmmss"
$OutFile = Join-Path $LogDir "${SafeTestName}_${Timestamp}.csv"

$Header = "host_receive_iso,host_receive_unix_ms,robot_time_us,robot_elapsed_us,sample,mode,bump_left,bump_right,left_encoder_deg,right_encoder_deg,delta_left_encoder_deg,delta_right_encoder_deg,left_distance_m,right_distance_m,x_m,y_m,theta_deg,left_cmd_milli,right_cmd_milli"

Set-Content -Path $OutFile -Value $Header -Encoding UTF8

$port = New-Object System.IO.Ports.SerialPort $PortName, $BaudRate, "None", 8, "One"
$port.NewLine = "`n"
$port.ReadTimeout = 1000
$port.DtrEnable = $true
$port.RtsEnable = $false

$LineCount = 0

Write-Host ""
Write-Host "Pololu bump + encoder maze logger"
Write-Host "Port:     $PortName"
Write-Host "Baud:     $BaudRate"
Write-Host "Test:     $TestName"
Write-Host "Log file: $OutFile"
Write-Host ""
Write-Host "CSV header has been written."
Write-Host "Press Ctrl+C to stop logging."
Write-Host ""

try {
    $port.Open()
    Write-Host "Opened $PortName."

    while ($true) {
        try {
            $line = $port.ReadLine().TrimEnd("`r", "`n")

            if ($line.Length -gt 0) {
                if ($line -match '^\d+,') {
                    $now = Get-Date
                    $hostIso = $now.ToString("o")
                    $hostUnixMs = ([DateTimeOffset]$now).ToUnixTimeMilliseconds()

                    $augmentedLine = "$hostIso,$hostUnixMs,$line"

                    Write-Host $augmentedLine
                    Add-Content -Path $OutFile -Value $augmentedLine -Encoding UTF8

                    $LineCount++
                }
                else {
                    Write-Host "Ignored non-data line: $line"
                }
            }
        }
        catch [System.TimeoutException] {
            Write-Host "." -NoNewline
        }
    }
}
catch {
    Write-Host ""
    Write-Host "Logger error:"
    Write-Host $_.Exception.Message
}
finally {
    if ($port.IsOpen) {
        $port.Close()
    }

    Write-Host ""
    Write-Host "Saved log to:"
    Write-Host $OutFile
    Write-Host "Data rows saved: $LineCount"
}