# 安全策略（Security Policy）

本文件适用于 rxsvision/rxs-algorithms 仓库及其所包含的独立 3D 测量算法模块。

## 支持的版本

| 版本/分支 | 支持状态 | 说明 |
|----------|---------|------|
| `main`   | ✅ 受支持 | 稳定发布分支，会接收安全修复 |
| `develop`| ✅ 受支持 | 日常开发分支，会尽快同步修复 |
| 历史 tag | ⚠️ 有限支持 | 仅对最新 tag 提供关键安全修复；建议升级到最新 tag |

## 如何报告安全漏洞

请 **不要** 通过公开 Issue 披露潜在安全漏洞。我们提供以下私有报告渠道：

1. **首选方式**：发送加密邮件至 `security@rxsvision.com`（请将 `[ ]` 中的占位符替换为实际邮箱）
2. **备用方式**：GitHub Private vulnerability reporting（每个仓库 Settings → Security → Private vulnerability reporting）

请在报告中尽可能包含以下内容：

- 漏洞类型（例如：缓冲区溢出、整数溢出、越界访问、内存泄漏等）
- 受影响的文件、函数或算法模块
- 可复现的环境（操作系统、编译器、PCL/OpenCV 版本）
- 最小可复现步骤或 PoC（如方便提供）
- 可能的修复建议（可选）
- 您的联系方式和希望被致谢的方式（可选，默认匿名）

## 响应流程

收到报告后，我们会按以下时间线处理：

| 阶段 | 时间目标 | 说明 |
|------|---------|------|
| 确认收到 | 2 个工作日内 | 发送收到回执，并分配跟踪编号 |
| 初步评估 | 5 个工作日内 | 确认影响范围、严重等级（Critical/High/Medium/Low） |
| 修复或缓解 | 视严重程度 | Critical/High：30 天内；Medium：60 天内；Low：90 天内 |
| 披露与致谢 | 修复后 | 发布安全公告（Security Advisory），并感谢报告者（如希望公开） |

## 漏洞协调披露（Coordinated Disclosure）

- 在修复版本正式发布前，请与我们保持私下沟通，不要公开漏洞细节。
- 修复发布后，我们会在 GitHub Security Advisories 页面发布公开的安全公告。
- 如需 CVE 编号，我们将协助申请。

## 安全扫描与自动化

本仓库已启用：

- **CodeQL**：每周一自动进行 C/C++ 静态安全扫描
- **Dependabot**：自动监控 GitHub Actions 依赖更新
- **Secret scanning + Push protection**：防止密钥被推送到仓库

## 许可证声明

本项目采用 **Business Source License 1.1 (BSL 1.1)**，在 Change Date（2026-06-30 版本为 2030-01-01）之前具有商业限制。安全漏洞报告与修复不影响许可证条款；请确保在报告或提交修复时遵守当前许可证约定。

## 联系方式

- 安全邮箱：`security@rxsvision.com`（请替换为实际邮箱）
- 组织：rxsvision（锐新视算法库重构项目）
