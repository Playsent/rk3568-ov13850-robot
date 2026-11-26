param(
    [Parameter(Mandatory)] [ipaddress]$ListenAddress,
    [string]$FFmpegPath = 'ffmpeg.exe',
    [ValidateRange(1, 1200)] [int]$WaitSeconds = 300,
    [ValidateRange(1024, 65535)] [int]$Port = 19353
)

$ErrorActionPreference = 'Stop'
if ($ListenAddress.AddressFamily -ne [Net.Sockets.AddressFamily]::InterNetwork -or
    $ListenAddress.Equals([IPAddress]::Any)) {
    throw 'Specify a local IPv4 address.'
}
$ffmpeg = (Get-Command -Name $FFmpegPath -CommandType Application -ErrorAction Stop).Source
$run = Join-Path $PSScriptRoot ('receive-' + (Get-Date -Format 'yyyyMMdd-HHmmss-fff'))
New-Item -ItemType Directory -Path $run | Out-Null
$url = 'rtmp://{0}:{1}/live/ov13850' -f $ListenAddress, $Port
$recording = Join-Path $run 'full-chain.mkv'
$receiveLog = Join-Path $run 'receive.log'
$decodeLog = Join-Path $run 'decode.log'
$argsForReceiver = @('-hide_banner', '-nostdin', '-n', '-listen', '1', '-timeout', $WaitSeconds,
    '-i', $url, '-map', '0:v:0', '-c:v', 'copy', 'full-chain.mkv')
$receiver = Start-Process -FilePath $ffmpeg -ArgumentList $argsForReceiver -WorkingDirectory $run `
    -RedirectStandardError $receiveLog -WindowStyle Hidden -PassThru
try {
    Write-Output "Receiver URL: $url"
    Write-Output "Results: $run"
    if (!$receiver.WaitForExit(($WaitSeconds + 100) * 1000)) {
        throw "FFmpeg receive process timed out. See $receiveLog"
    }
    Write-Output "receiver_exit=$($receiver.ExitCode)"
    if (!(Test-Path -LiteralPath $recording -PathType Leaf) -or
        (Get-Item -LiteralPath $recording).Length -eq 0) {
        throw "No recording was received. See $receiveLog"
    }
} finally {
    if (!$receiver.HasExited) { Stop-Process -Id $receiver.Id }
}
$progress = & $ffmpeg -hide_banner -nostdin -nostats -xerror -progress pipe:1 -i $recording -map '0:v:0' -f null NUL 2> $decodeLog
$decodeExit = $LASTEXITCODE
$progress | Set-Content -LiteralPath (Join-Path $run 'decode-progress.txt')
if ($decodeExit -ne 0 -or !($progress -contains 'progress=end')) {
    throw "Received recording did not decode cleanly. See $decodeLog"
}
$frames = $progress | Select-String '^frame=(\d+)$' | Select-Object -Last 1
$time = $progress | Select-String '^out_time=(.*)$' | Select-Object -Last 1
if (!$frames -or [int]$frames.Matches[0].Groups[1].Value -eq 0) {
    throw "No video frames were decoded. See $decodeLog"
}
$stream = Get-Content -LiteralPath $decodeLog | Select-String 'Stream #0:0.*Video: h264.*1280x720' | Select-Object -First 1
Write-Output "decoded_frames=$($frames.Matches[0].Groups[1].Value)"
if ($time) { Write-Output $time.Line }
if (!$stream) { throw "Received video did not report H.264 1280x720. See $decodeLog" }
Write-Output $stream.Line
Write-Output 'WINDOWS_RTMP_DECODE_OK'
