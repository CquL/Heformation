---
name: local-codex-remote-ssh-workspace
description: >
  在 macOS 本地使用 Codex 桌面应用，通过 SSH 密钥和 ~/.ssh/config 访问远程 Linux
  服务器上的指定项目目录；Codex 登录凭据只保留在本地，不要求在远程服务器登录 Codex。
version: 1.0
---

# Local Codex → Remote SSH Workspace

## 目标

建立以下工作模式：

```text
Mac 本地
├── Codex Desktop（登录个人 ChatGPT/Codex 账号）
├── 本地工作目录（只放 AGENTS.md 等控制文件）
└── ssh <HOST_ALIAS>
      ↓
远程 Linux 服务器
└── <REMOTE_PROJECT_DIR>
    ├── 源代码
    ├── Git 仓库
    ├── Python/训练环境
    └── GPU / 数据
```

核心原则：

- Codex 账号只登录在本地 Mac。
- 不在远程服务器执行 `codex login`。
- 不把本地 `~/.codex/auth.json` 或其他 Codex 凭据复制到服务器。
- 本地 Codex 通过普通 SSH 命令读取、修改、测试远程项目。
- 远程服务器是项目文件的 source of truth。

---

## 适用场景

当满足以下条件时使用此 skill：

- 用户在 Mac 上使用 Codex Desktop。
- 项目实际位于远程 Linux/GPU 服务器。
- 用户希望 Codex 能修改远程项目，但不希望个人 Codex 凭据落到服务器。
- 服务器支持 SSH 登录。
- 用户可以在 Mac 上建立 SSH key 登录。

---

## 参数

执行前先确定：

```text
REMOTE_HOST       = 远程服务器 IP 或域名
REMOTE_PORT       = SSH 端口
REMOTE_USER       = SSH 用户名
HOST_ALIAS        = 本地 SSH 别名，例如 atm-gpu
REMOTE_PROJECT_DIR= 远程项目目录
LOCAL_WORKSPACE   = 本地 Codex 控制目录
```

示例：

```text
REMOTE_HOST        = 203.0.113.10
REMOTE_PORT        = 50054
REMOTE_USER        = root
HOST_ALIAS         = atm-gpu
REMOTE_PROJECT_DIR = /data/user/codes/PROJECT
LOCAL_WORKSPACE    = ~/codex-project
```

> 不要把密码、私钥、OAuth token 写入 AGENTS.md、聊天记录或项目仓库。

---

## 步骤 1：确认从 Mac 能 SSH 到服务器

必须在 **Mac 本地终端** 执行，而不是服务器终端：

```bash
ssh -p <REMOTE_PORT> <REMOTE_USER>@<REMOTE_HOST>
```

如果能进入服务器，先退出：

```bash
exit
```

---

## 步骤 2：配置 Mac 的 SSH 密钥

### 2.1 检查现有密钥

在 Mac 本地执行：

```bash
ls -l ~/.ssh/id_ed25519 ~/.ssh/id_ed25519.pub
```

如果已存在，不要直接覆盖。

如果不存在，再生成：

```bash
ssh-keygen -t ed25519
```

当提示保存位置时，通常直接回车使用：

```text
~/.ssh/id_ed25519
```

如果已有同名密钥并询问：

```text
Overwrite (y/n)?
```

选择：

```text
n
```

避免破坏已经用于 GitHub、其他服务器等场景的旧密钥。

### 2.2 把 Mac 公钥加入服务器

在 Mac 本地执行：

```bash
cat ~/.ssh/id_ed25519.pub | ssh -p <REMOTE_PORT> <REMOTE_USER>@<REMOTE_HOST> "mkdir -p ~/.ssh && chmod 700 ~/.ssh && cat >> ~/.ssh/authorized_keys && chmod 600 ~/.ssh/authorized_keys"
```

此时通常需要最后输入一次服务器密码。

### 2.3 验证免密 SSH

```bash
ssh -p <REMOTE_PORT> <REMOTE_USER>@<REMOTE_HOST>
```

如果不再要求服务器密码，说明成功。

然后退出：

```bash
exit
```

---

## 步骤 3：配置 SSH 别名

编辑 Mac 本地：

```bash
nano ~/.ssh/config
```

加入：

```sshconfig
Host <HOST_ALIAS>
    HostName <REMOTE_HOST>
    User <REMOTE_USER>
    Port <REMOTE_PORT>
    IdentityFile ~/.ssh/id_ed25519
```

例如：

```sshconfig
Host atm-gpu
    HostName 203.0.113.10
    User root
    Port 50054
    IdentityFile ~/.ssh/id_ed25519
```

测试：

```bash
ssh <HOST_ALIAS>
```

成功后退出：

```bash
exit
```

---

## 步骤 4：验证指定远程项目目录

在 Mac 本地执行：

```bash
ssh <HOST_ALIAS> "cd <REMOTE_PROJECT_DIR> && pwd && ls -la && git status"
```

如果项目不是 Git 仓库，可去掉 `git status`。

常用验证：

```bash
ssh <HOST_ALIAS> "cd <REMOTE_PROJECT_DIR> && pwd"
ssh <HOST_ALIAS> "cd <REMOTE_PROJECT_DIR> && ls -la"
ssh <HOST_ALIAS> "cd <REMOTE_PROJECT_DIR> && git status"
ssh <HOST_ALIAS> "nvidia-smi"
```

---

## 步骤 5：创建本地 Codex 控制目录

这个目录不是远程代码副本，只用于给 Codex Desktop 一个本地 workspace 和项目规则。

```bash
mkdir -p <LOCAL_WORKSPACE>
cd <LOCAL_WORKSPACE>
nano AGENTS.md
```

例如：

```bash
mkdir -p ~/codex-project
cd ~/codex-project
nano AGENTS.md
```

---

## 步骤 6：AGENTS.md 模板

将以下内容写入 `<LOCAL_WORKSPACE>/AGENTS.md`：

```text
This project lives on a remote Linux server.

Remote SSH host:
<HOST_ALIAS>

Remote project directory:
<REMOTE_PROJECT_DIR>

Important rules:

1. Do not assume project files are local.
2. Read, search, modify, run, test, and inspect the project through SSH.
3. Use commands in this form:
   ssh <HOST_ALIAS> "cd <REMOTE_PROJECT_DIR> && <command>"
4. For multi-line or complex shell operations, still execute them on <HOST_ALIAS>.
5. Never run `codex login`, `codex logout`, or modify ~/.codex on the remote server.
6. Do not copy local Codex/ChatGPT authentication files to the remote server.
7. Inspect relevant remote files before modifying them.
8. Prefer safe edits. Preserve unrelated user changes.
9. Run tests, Python programs, training jobs, and GPU commands on the remote server.
10. The remote server is the source of truth for this project.
11. Before making changes, check `git status`.
12. After making changes, report which remote files were modified and what verification was run.

Examples:

List project files:
ssh <HOST_ALIAS> "cd <REMOTE_PROJECT_DIR> && ls -la"

Git status:
ssh <HOST_ALIAS> "cd <REMOTE_PROJECT_DIR> && git status"

Read a file:
ssh <HOST_ALIAS> "cd <REMOTE_PROJECT_DIR> && sed -n '1,220p' path/to/file.py"

Search:
ssh <HOST_ALIAS> "cd <REMOTE_PROJECT_DIR> && rg 'keyword'"

Run Python:
ssh <HOST_ALIAS> "cd <REMOTE_PROJECT_DIR> && python script.py"

Check GPU:
ssh <HOST_ALIAS> "nvidia-smi"
```

---

## 步骤 7：在 Codex Desktop 中打开

不要在服务器运行 Codex。

在 Mac 上打开 **Codex Desktop**：

1. 选择打开本地文件夹。
2. 选择 `<LOCAL_WORKSPACE>`。
3. 确认其中存在 `AGENTS.md`。
4. 在 Codex 中发出远程任务。

推荐的首次测试提示：

```text
请读取当前工作区的 AGENTS.md。

通过 ssh <HOST_ALIAS> 进入远程项目 <REMOTE_PROJECT_DIR>，
先执行 pwd、ls -la 和 git status，确认你能够访问项目。

暂时不要修改任何文件。
```

预期 Codex 执行：

```bash
ssh <HOST_ALIAS> "cd <REMOTE_PROJECT_DIR> && pwd && ls -la && git status"
```

---

## 日常使用方式

### 查看远程文件

```text
读取远程项目中的 src/train.py，解释训练入口，不要修改文件。
```

Codex 应通过类似命令完成：

```bash
ssh <HOST_ALIAS> "cd <REMOTE_PROJECT_DIR> && sed -n '1,260p' src/train.py"
```

### 修改远程代码

```text
先检查 git status，然后修改远程项目中的 xxx.py。
只修改与这个需求相关的内容，修改后运行对应测试。
```

### 运行训练/测试

```text
在远程服务器项目目录中运行 pytest。
```

或：

```text
在远程服务器上检查 GPU，并运行训练脚本。
```

---

## 安全要求

### 不要做

不要在远程服务器执行：

```bash
codex login
```

不要复制：

```text
~/.codex/auth.json
```

不要覆盖已有 SSH 私钥：

```text
~/.ssh/id_ed25519
```

不要把以下内容提交到 Git：

- SSH 私钥
- 服务器密码
- Codex OAuth token
- API key
- `auth.json`

### 推荐做

如果服务器密码曾经出现在截图、聊天或其他可见位置，应立即更换：

```bash
passwd
```

SSH key 配置成功后，可进一步考虑关闭 root 密码登录、使用普通用户 + sudo，但这属于服务器加固的后续步骤。

---

## 常见问题排查

### 1. `ssh: Could not resolve hostname <HOST_ALIAS>`

检查：

```bash
cat ~/.ssh/config
```

然后：

```bash
ssh -v <HOST_ALIAS>
```

确认 Host 别名、IP、端口和用户名正确。

### 2. 仍然要求输入服务器密码

检查 Mac 公钥：

```bash
cat ~/.ssh/id_ed25519.pub
```

检查服务器端：

```bash
cat ~/.ssh/authorized_keys
```

以及权限：

```bash
chmod 700 ~/.ssh
chmod 600 ~/.ssh/authorized_keys
```

### 3. Codex Desktop 本地聊天正常，但 Remote SSH workspace 报 OAuth/token_revoked

不要把它当成普通 SSH 故障。

如果目标是“Codex 凭据只留在本地”，不要使用需要远程 Codex OAuth 的工作模式，改用本 skill 的：

```text
本地 Codex Desktop
→ 本地 workspace + AGENTS.md
→ 普通 ssh <HOST_ALIAS> "..."
→ 远程项目
```

### 4. 远程服务器上的 `codex` 登录的是别的账号

本 skill 不依赖远程服务器上的 Codex 登录。

不要修改服务器现有：

```text
~/.codex
```

也不要执行：

```bash
codex logout
codex login
```

除非服务器管理员明确要求。

### 5. Codex 执行 SSH 时要求批准

这是本地 Codex 的执行/网络权限控制。

首次 SSH 操作可能需要在 Codex Desktop 中批准。批准的是本地 Codex 发起 SSH 命令，不代表在远程服务器登录 Codex。

### 6. 命令在本地执行了，而不是服务器

检查 `AGENTS.md` 是否明确要求所有项目命令使用：

```bash
ssh <HOST_ALIAS> "cd <REMOTE_PROJECT_DIR> && <command>"
```

并在任务中明确写：

```text
所有项目读写和运行均通过 ssh <HOST_ALIAS> 完成。
```

---

## 验收清单

完成后逐项确认：

- [ ] Mac 可以 `ssh <HOST_ALIAS>` 进入服务器。
- [ ] SSH 登录不需要输入服务器密码。
- [ ] `ssh <HOST_ALIAS> "cd <REMOTE_PROJECT_DIR> && pwd"` 正确。
- [ ] `git status` 能从 Mac 通过 SSH 获取。
- [ ] 本地已创建 `<LOCAL_WORKSPACE>/AGENTS.md`。
- [ ] Codex Desktop 打开的是本地 `<LOCAL_WORKSPACE>`。
- [ ] Codex 能通过 SSH 读取远程文件。
- [ ] Codex 能通过 SSH 修改远程文件。
- [ ] 测试/训练实际在远程服务器运行。
- [ ] 远程服务器没有登录个人 Codex 账号。
- [ ] 本地 Codex 凭据没有复制到服务器。

---

## 一句话工作流

以后每次使用时：

```text
打开 Codex Desktop
→ 打开本地控制目录
→ Codex 读取 AGENTS.md
→ 通过 ssh <HOST_ALIAS> 操作 <REMOTE_PROJECT_DIR>
→ 代码和计算都留在服务器
→ Codex 登录只留在本地 Mac
```
