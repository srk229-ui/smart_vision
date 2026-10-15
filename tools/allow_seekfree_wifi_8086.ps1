# 仅放行当前手机热点中逐飞助手的 TCP 8086 入站通信。
$ErrorActionPreference = 'Stop'
$resultPath = Join-Path $PSScriptRoot 'seekfree_firewall_result.json'
$ruleName = 'SeekFree-WIFI-Test-TCP-8086'
$assistantPath = 'D:\镜头组\tools\seekfree_assistant\【软件】逐飞助手V1版\逐飞助手V1.2.7.exe'
try {
    $identity = [Security.Principal.WindowsIdentity]::GetCurrent()
    $principal = New-Object Security.Principal.WindowsPrincipal($identity)
    if (-not $principal.IsInRole([Security.Principal.WindowsBuiltInRole]::Administrator)) {
        throw '需要管理员权限才能添加 Windows 防火墙规则。'
    }
    if (-not (Test-Path -LiteralPath $assistantPath)) { throw '逐飞助手程序路径不存在。' }
    $listener = Get-NetTCPConnection -LocalAddress '10.138.155.234' -LocalPort 8086 -State Listen -ErrorAction Stop | Select-Object -First 1
    $process = Get-CimInstance Win32_Process -Filter ('ProcessId=' + $listener.OwningProcess)
    if ($process.ExecutablePath -ne $assistantPath) { throw 'TCP 8086 当前监听程序不是指定的逐飞助手。' }
    $rule = Get-NetFirewallRule -Name $ruleName -ErrorAction SilentlyContinue
    if ($null -eq $rule) {
        $rule = New-NetFirewallRule -Name $ruleName -DisplayName '逐飞助手 WiFi 测试 TCP 8086（当前手机热点）' -Description '仅允许逐飞助手 V1.2.7 在 WLAN 10.138.155.234 的 TCP 8086 接收当前手机热点子网 10.138.155.0/24 的入站连接。' -Direction Inbound -Action Allow -Enabled True -Profile Public -Program $assistantPath -Protocol TCP -LocalPort 8086 -LocalAddress '10.138.155.234' -RemoteAddress '10.138.155.0/24' -InterfaceAlias 'WLAN'
    }
    $port = $rule | Get-NetFirewallPortFilter
    $address = $rule | Get-NetFirewallAddressFilter
    $application = $rule | Get-NetFirewallApplicationFilter
    [ordered]@{
        Success = $true
        RuleName = $rule.Name
        Enabled = [string]$rule.Enabled
        Action = [string]$rule.Action
        Profile = [string]$rule.Profile
        Program = $application.Program
        Protocol = [string]$port.Protocol
        LocalPort = [string]$port.LocalPort
        LocalAddress = @($address.LocalAddress)
        RemoteAddress = @($address.RemoteAddress)
        RollbackCommand = "Remove-NetFirewallRule -Name 'SeekFree-WIFI-Test-TCP-8086'"
    } | ConvertTo-Json -Depth 4 | Set-Content -LiteralPath $resultPath -Encoding UTF8
} catch {
    [ordered]@{ Success = $false; Error = $_.Exception.Message } | ConvertTo-Json | Set-Content -LiteralPath $resultPath -Encoding UTF8
    exit 1
}
