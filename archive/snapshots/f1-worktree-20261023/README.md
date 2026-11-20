# F1 worktree 未提交改动快照（2026-10-23）

来源：本地 worktree `.worktrees/f1`（分支 `f1-arm-interface`，HEAD `acbf725`，2026-10-23「F1 对外服务与电脑端网页」）上没有提交的改动，2026-11-20 整理仓库时导出。

- `tracked.patch`：已跟踪文件的改动（`src/web/*`、`tests/test_archive.py`、`tests/test_timers.py`）
- `untracked.patch`：新文件（`src/web/backends/`、`src/web/task.py`、`src/web/js/components.js`、三个测试、doc 21 草稿、`scripts/README.md`），清单见 `untracked.txt`

路径是 2026-10-23 的旧布局（`src/web/`，还没有 `app/`）。这些思路后来由 PR #1 收敛后的版本和 `app/src/web/` 取代，**不再维护**；只在需要找回当时的写法时参考：

```bash
git worktree add /tmp/f1 acbf725 && cd /tmp/f1 && git apply <本目录>/tracked.patch <本目录>/untracked.patch
```
