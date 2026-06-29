param(
    [string]$PortName = "COM5",
    [int]$BaudRate = 115200,
    [string]$TestName = "pololu_line_encoder_imu_track_follow"
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

# The robot prints the CSV header first. This script prepends host timing
# columns to that robot header, so it stays correct when the LF schema changes.
$HeaderWritten = $false

$port = New-Object System.IO.Ports.SerialPort $PortName, $BaudRate, "None", 8, "One"
$port.NewLine = "`n"
$port.ReadTimeout = 1000
$port.DtrEnable = $true
$port.RtsEnable = $false

$LineCount = 0

Write-Host ""
Write-Host "Pololu line + encoder + IMU logger"
Write-Host "Port:     $PortName"
Write-Host "Baud:     $BaudRate"
Write-Host "Test:     $TestName"
Write-Host "Log file: $OutFile"
Write-Host ""
Write-Host "Waiting for robot CSV header."
Write-Host "For TrackFollowSolution_encoder_imu.lf: after calibration, press A to start, C to stop, then A to dump."
Write-Host "Press Ctrl+C to stop logging."
Write-Host ""

try {
    $port.Open()
    Write-Host "Opened $PortName."

    while ($true) {
        try {
            $line = $port.ReadLine().TrimEnd("`r", "`n")

            if ($line.Length -gt 0) {
                # Robot CSV header. Do not hard-code robot columns here.
                if ($line -match '^robot_time_us,') {
                    $Header = "host_receive_iso,host_receive_unix_ms,$line"
                    Set-Content -Path $OutFile -Value $Header -Encoding UTF8
                    $HeaderWritten = $true
                    Write-Host $Header
                }
                # Robot CSV data row.
                elseif ($line -match '^\d+,') {
                    if (-not $HeaderWritten) {
                        Write-Host "Ignored data row before CSV header: $line"
                        continue
                    }

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

    if (-not $HeaderWritten) {
        Write-Host "Warning: no robot CSV header was received, so the output file may be empty or missing."
    }
}
