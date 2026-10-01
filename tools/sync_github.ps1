param(
    [Parameter(Mandatory = $true)]
    [string]$Message
)

$ErrorActionPreference = 'Stop'
$projectRoot = Split-Path -Parent $PSScriptRoot
Set-Location -LiteralPath $projectRoot

function Invoke-RepoGit {
    & git -c "safe.directory=$projectRoot" @args
}

$remoteUrl = Invoke-RepoGit remote get-url origin 2>$null
if ($LASTEXITCODE -ne 0 -or [string]::IsNullOrWhiteSpace($remoteUrl)) {
    throw '未配置 origin；请先创建并连接 GitHub 仓库。'
}

# Fetch/pull before every local submission. If the remote is still empty,
# there is no upstream commit to rebase onto.
Invoke-RepoGit fetch origin
if ($LASTEXITCODE -ne 0) { throw 'git fetch 失败，请检查网络和 GitHub 登录状态。' }

Invoke-RepoGit show-ref --verify --quiet refs/remotes/origin/main
if ($LASTEXITCODE -eq 0) {
    Invoke-RepoGit pull --rebase --autostash origin main
    if ($LASTEXITCODE -ne 0) { throw 'git pull --rebase 失败；请先解决冲突。' }
}

Invoke-RepoGit add -A
if ($LASTEXITCODE -ne 0) { throw 'git add 失败。' }
Invoke-RepoGit diff --cached --quiet
if ($LASTEXITCODE -eq 0) {
    Write-Host '没有新的变更需要提交。'
    exit 0
}

Invoke-RepoGit commit -m $Message
if ($LASTEXITCODE -ne 0) { throw 'git commit 失败。' }
Invoke-RepoGit push -u origin main
if ($LASTEXITCODE -ne 0) { throw 'git push 失败；本地提交仍然保留。' }
