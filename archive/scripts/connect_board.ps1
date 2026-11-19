[CmdletBinding()]
param(
    [string]$HostName = '192.168.137.100',
    [string]$UserName = 'root',
    [int]$Port = 22
)

$probe = Test-NetConnection -ComputerName $HostName -Port $Port -InformationLevel Quiet
if (-not $probe) {
    throw "SSH 端口不可达：${HostName}:${Port}"
}

& ssh -p $Port "${UserName}@${HostName}"
exit $LASTEXITCODE
