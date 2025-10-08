 # 環境建置（Environment Setup）

為了開發與測試，我們將依 **NIST** 建議使用 Docker 建立官方 **ARIAC 2025** 模擬環境。ARIAC 2025 基於 **Ubuntu 24.04** 上的 **ROS 2 Jazzy** 與 Gazebo 模擬環境（來源見 nist.gov 連結）。開發工具以 VS Code（Remote Containers）與 Git 為主，並在本機 Docker 容器中部署與執行解決方案：

- **安裝 Docker 與 NVIDIA 支援：** 在 Linux 主機上安裝 Docker Engine 與 NVIDIA Container Toolkit（用於 GPU 加速）[pages.nist.gov](https://pages.nist.gov/ARIAC_docs/en/latest/pages/startup.html#:~:text=Attention)[pages.nist.gov](https://pages.nist.gov/ARIAC_docs/en/latest/pages/startup.html#:~:text=)。確認 Docker 已啟動；若使用 NVIDIA GPU，請執行 `sudo nvidia-ctk runtime configure --runtime=docker` 設定 runtime[pages.nist.gov](https://pages.nist.gov/ARIAC_docs/en/latest/pages/startup.html#:~:text=After%20installation%2C%20run%20the%20following,command)。
- **拉取 ARIAC 映像：** 從 Docker Hub 取得官方 ARIAC 2025 映像：

  ```bash
  docker pull nistariac/ariac2025:latest
  ```

  該映像包含所有競賽相依與 ROS 2 套件。

- **下載示例倉庫：** 取得 ARIAC example team 作為模板與示範：

  ```bash
  git clone https://github.com/usnistgov/ariac_example_team.git ~/ariac_example_team
  ```

  內含 ROS 2（Python）範例程式與示範組態。

- **使用 Docker Compose 啟動容器：** 透過示例倉庫的 `docker-compose.yml` 啟動模擬容器。具有 NVIDIA GPU 的機器可執行 `docker compose up ariac_nvidia`，無 GPU 則執行 `docker compose up ariac`[pages.nist.gov](https://pages.nist.gov/ARIAC_docs/en/latest/pages/startup.html#:~:text=4,Docker%20Compose)。這會建立（例如命名為 `example_team`）的 ARIAC 環境容器。
- **啟動模擬環境：** 在新終端機（容器內）啟動 ARIAC Gazebo 與示例組態：

  ```bash
  docker exec -it example_team bash
  ros2 launch ariac_gz ariac.launch.py user_config:=/team_ws/src/example_team/config/example_team_config.yaml trial_config:=/team_ws/src/example_team/config/trials/LHAF9835.yaml
  ```

  這會載入模擬工廠環境並讀入示例試題設定[pages.nist.gov](https://pages.nist.gov/ARIAC_docs/en/latest/pages/startup.html#:~:text=1,in%20the%20container)。

- **啟動 ARIAC Web App：** 於另一個容器終端機啟動 ARIAC App GUI：

  ```bash
  ros2 run ariac_app app
  ```

  啟動後可於 `http://localhost:8080` 存取 NiceGUI 介面[pages.nist.gov](https://pages.nist.gov/ARIAC_docs/en/latest/pages/startup.html#:~:text=NiceGUI%20ready%20to%20go%20on,x%3A8080)，用以建立自訂試題、管理執行與檢視結果。

- **本機開發替代方案：** （選擇性）可在 Ubuntu 24.04 + ROS 2 Jazzy 原生建置 ARIAC 原始碼；但為了可重現性與最終提交需求，建議以 Docker 為主[pages.nist.gov](https://pages.nist.gov/ARIAC_docs/en/latest/pages/startup.html#:~:text=Docker)。

完成上述步驟後，我們將擁有可重現的開發環境。所有程式碼將放在容器內的 ROS 2 工作區（`/team_ws`），可透過 VS Code 編輯。版本控制以 GitHub 為主，並以容器內建置與執行的工作流程進行即時測試。

---

# 競賽總覽與需求（Competition Overview and Requirements）

*概述：ARIAC 2025 模擬工廠包含多個工作站（檢測、組裝、輸送帶與 AGV）。目標是在動態的 EV 電池組裝情境中驗證機器人靈活性與可靠性。*

**Agile Robotics for Industrial Automation Competition (ARIAC) 2025** 模擬電動車電池生產工廠。隊伍需為多台機器人編程，以高敏捷度與高可靠性完成製造任務[pages.nist.gov](https://pages.nist.gov/ARIAC_docs/en/latest/index.html#:~:text=The%20Agile%20Robotics%20for%20Industrial,on%20adaptability%2C%20autonomy%2C%20and%20efficiency)[pages.nist.gov](https://pages.nist.gov/ARIAC_docs/en/latest/index.html#:~:text=,eligible%20teams)。情境分為兩大任務：

- **任務 1：檢測與配料（Inspection and Kit Building）。** 需檢測單顆電池是否有瑕疵，並將合格電池組成 **kit**（四顆良品）[pages.nist.gov](https://pages.nist.gov/ARIAC_docs/en/latest/pages/scenario.html#:~:text=During%20a%20competition%20run%2C%20teams,as%20determined%20by%20the%20trial)[pages.nist.gov](https://pages.nist.gov/ARIAC_docs/en/latest/pages/scenario.html#:~:text=The%20scenario%20is%20broken%20down,into%20two%20main%20tasks)。子步驟：
  - *1a 物理檢測：* 電池（Li-Ion 或 NiMH）穿越檢測輸送帶；以 3D LIDAR 重建幾何檢測瑕疵（凹痕、刮痕、膨脹）[pages.nist.gov](https://pages.nist.gov/ARIAC_docs/en/latest/pages/scenario.html#:~:text=For%20each%20cell%2C%20teams%20must,completely%20cylindrical%20with%20smooth%20surfaces)。對每顆電池回報合格/不合格與缺陷類型[pages.nist.gov](https://pages.nist.gov/ARIAC_docs/en/latest/pages/scenario.html#:~:text=After%20the%20inspection%20is%20complete%2C,The%20inspection%20report%20contains)。*不合格* 定義為表面異常 ≥ 2 mm[pages.nist.gov](https://pages.nist.gov/ARIAC_docs/en/latest/pages/scenario.html#:~:text=,beyond%20the%20expected%20cell%20surface)。
  - *1b 輸送帶拾取：* 合格電池通過檢測站後，由 **檢測機器人 1（UR3e + Robotiq 2F-85）** 自移動輸送帶上抓取並放入**電壓測試器**[pages.nist.gov](https://pages.nist.gov/ARIAC_docs/en/latest/pages/scenario.html#:~:text=Step%201b%3A%20Conveyor%20Pickup)。缺陷電池須忽略或轉棄，避免進入 kit（否則扣分）[pages.nist.gov](https://pages.nist.gov/ARIAC_docs/en/latest/pages/scenario.html#:~:text=If%20a%20non,team%20will%20incur%20a%20penalty)。
  - *1c 電壓檢測：* 測試器於 ROS topic 發佈電壓；我們需取樣並判定是否在允收範圍[pages.nist.gov](https://pages.nist.gov/ARIAC_docs/en/latest/pages/scenario.html#:~:text=Step%201c%3A%20Voltage%20Inspection)。
  - *1d AGV 擺放：* **檢測機器人 2（UR5e）** 自測試器取出電池；若電壓超限（相對名義值 ±0.2 V），丟棄於回收桶；若合格則放入 AGV 的 kit 托盤[pages.nist.gov](https://pages.nist.gov/ARIAC_docs/en/latest/pages/scenario.html#:~:text=Step%201d%3A%20AGV%20Placement)。需四顆同類型良品完成一個 kit。
  - *1e AGV 移動與提交：* kit 滿四顆後，指派 AGV 至 **Shipping Station**，再呼叫服務 **提交 kit** 以驗證（類型一致、電壓合格、總電壓正確）[pages.nist.gov](https://pages.nist.gov/ARIAC_docs/en/latest/pages/scenario.html#:~:text=To%20complete%20a%20kitting%20order,at%20the%20recycling%20station%20a)。通過則計分並清空托盤；失敗則需送回收站清空後再用[pages.nist.gov](https://pages.nist.gov/ARIAC_docs/en/latest/pages/scenario.html#:~:text=To%20complete%20a%20kitting%20order,AGV%20to%20be%20used%20again)。
- **任務 2：模組組裝（Module Assembly）。** 以完成的 kits 組裝更大的電池 **module**（含上下殼與焊點）[pages.nist.gov](https://pages.nist.gov/ARIAC_docs/en/latest/pages/scenario.html#:~:text=A%20kit%20is%20a%20grouping,cells%20placed%20onto%20a%20tray)。若情境需要模組訂單才會進行。子步驟：
  - *2a Cell 安裝：* AGV 到達 **Assembly Station** 後，先以服務呼叫插入 **底殼**[pages.nist.gov](https://pages.nist.gov/ARIAC_docs/en/latest/pages/scenario.html#:~:text=When%20an%20AGV%20arrives%20at,spawn%20in%20the%20same%20place)。**Assembly Robot 1（UR5e）** 將四顆 cell 自 AGV 置入底殼槽位，並確保 **極性方向** 正確（殼上有「+ / –」標示）[pages.nist.gov](https://pages.nist.gov/ARIAC_docs/en/latest/pages/scenario.html#:~:text=Cells%20should%20be%20picked%20using,grasp%20of%20the%20cell)。需要時可利用 AGV 托盤中央對位孔 re-grasp[pages.nist.gov](https://pages.nist.gov/ARIAC_docs/en/latest/pages/scenario.html#:~:text=Cells%20should%20be%20picked%20using,grasp%20of%20the%20cell)。放置後 cell 會卡入定位[pages.nist.gov](https://pages.nist.gov/ARIAC_docs/en/latest/pages/scenario.html#:~:text=Note)。
  - *2b 換工具：* **Assembly Robot 2（UR10e + tool changer）** 需裝上 **VG2**（雙吸盤）以搬運 top shell。機械手需對準工具座並呼叫 `attach_tool` 服務上鎖，避免碰撞[pages.nist.gov](https://pages.nist.gov/ARIAC_docs/en/latest/pages/scenario.html#:~:text=Assembly%20robot%202%20is%20equipped,the%20tool%20stand%20without%20colliding)[pages.nist.gov](https://pages.nist.gov/ARIAC_docs/en/latest/pages/scenario.html#:~:text=Note)。
  - *2c 安裝上殼：* 組裝輸送帶有三段可驅動；top shell 以服務呼叫隨機姿態產生於工作台[pages.nist.gov](https://pages.nist.gov/ARIAC_docs/en/latest/pages/scenario.html#:~:text=Unlike%20the%20inspection%20conveyor%20which,until%20it%20reaches%20section%203)[pages.nist.gov](https://pages.nist.gov/ARIAC_docs/en/latest/pages/scenario.html#:~:text=The%20next%20step%20is%20to,it%20onto%20the%20partial%20module)。須以感測器定位其姿態，Robot 2（VG2）取放並對齊到底殼[pages.nist.gov](https://pages.nist.gov/ARIAC_docs/en/latest/pages/scenario.html#:~:text=The%20next%20step%20is%20to,it%20onto%20the%20partial%20module)。兩個吸盤需接觸才能正確提起[pages.nist.gov](https://pages.nist.gov/ARIAC_docs/en/latest/pages/scenario.html#:~:text=Image%3A%20)。
  - *2d 上表面焊接：* 將模組送至龍門焊接機下方，透過服務呼叫完成四個焊點[pages.nist.gov](https://pages.nist.gov/ARIAC_docs/en/latest/pages/scenario.html#:~:text=With%20the%20top%20shell%20in,performed%20through%20a%20service%20call)。
  - *2e 模組翻轉：* Robot 2 放回 VG2、改裝 **VG4（四吸盤）**[pages.nist.gov](https://pages.nist.gov/ARIAC_docs/en/latest/pages/scenario.html#:~:text=After%20completing%20the%20top%20welds%2C,stand%20and%20pick%20up%20VG4)，自側面吸取後 **翻轉 180°** 放回輸送帶[pages.nist.gov](https://pages.nist.gov/ARIAC_docs/en/latest/pages/scenario.html#:~:text=After%20completing%20the%20top%20welds%2C,stand%20and%20pick%20up%20VG4)[pages.nist.gov](https://pages.nist.gov/ARIAC_docs/en/latest/pages/scenario.html#:~:text=Warning)。
  - *2f 下表面焊接與提交：* 完成剩餘兩個焊點後，移至輸出站並呼叫 **提交 module** 服務完成評分與移除[pages.nist.gov](https://pages.nist.gov/ARIAC_docs/en/latest/pages/scenario.html#:~:text=Step%202f%3A%20Bottom%20Welds)[pages.nist.gov](https://pages.nist.gov/ARIAC_docs/en/latest/pages/scenario.html#:~:text=The%20final%20step%20involves%20performing,module%20service%20can%20be%20called)。

兩任務中皆需避免碰撞（機器人對機器人、機器人對環境），否則重罰[pages.nist.gov](https://pages.nist.gov/ARIAC_docs/en/latest/pages/scenario.html#:~:text=Warning)。安全間隙與軌跡至關重要。

**敏捷挑戰（Agility Challenges）：** 比賽會注入突發事件以測韌性[pages.nist.gov](https://pages.nist.gov/ARIAC_docs/en/latest/pages/challenges.html#:~:text=The%20agility%20challenges%20test%20teams%E2%80%99,performance%20and%20achieving%20competitive%20scores)：
- *輸送帶故障：* 檢測輸送帶可能**暫停**一段時間；需監測狀態 topic 並暫停假設其在移動的動作[pages.nist.gov](https://pages.nist.gov/ARIAC_docs/en/latest/pages/challenges.html#:~:text=Conveyor%20Malfunction)。
- *電壓測試器故障：* 其中一台可能**停止輸出**讀值；需偵測該 topic 無資料並切換使用另一台，或暫存等待恢復[pages.nist.gov](https://pages.nist.gov/ARIAC_docs/en/latest/pages/challenges.html#:~:text=Voltage%20Tester%20Malfunction)[pages.nist.gov](https://pages.nist.gov/ARIAC_docs/en/latest/pages/challenges.html#:~:text=1,reference)。
- *真空工具故障：* 指定抓取次數會**故意失敗**；需偵測失敗並重試（可能重新定位後再抓）[pages.nist.gov](https://pages.nist.gov/ARIAC_docs/en/latest/pages/challenges.html#:~:text=When%20the%20vacuum%20tool%20malfunction,will%20fail%20during%20grasp%20attempts)[pages.nist.gov](https://pages.nist.gov/ARIAC_docs/en/latest/pages/challenges.html#:~:text=1,reference)。
- *高優先訂單：* 會隨機發布**NiMH kit** 緊急訂單；需立即切換餵料、優先完成並用特別服務提交（帶正確 order ID）以爭取時間獎勵[pages.nist.gov](https://pages.nist.gov/ARIAC_docs/en/latest/pages/challenges.html#:~:text=%2A%20Vacuum%20Tool%20Malfunction%20,gripper%20fails%20to%20grasp%20objects)[pages.nist.gov](https://pages.nist.gov/ARIAC_docs/en/latest/pages/challenges.html#:~:text=When%20a%20high%20priority%20order,submit%20this%20urgent%20kit%20request)[pages.nist.gov](https://pages.nist.gov/ARIAC_docs/en/latest/pages/challenges.html#:~:text=1,reference)[pages.nist.gov](https://pages.nist.gov/ARIAC_docs/en/latest/pages/challenges.html#:~:text=4,AGV%20to%20the%20shipping%20location)。

**感測器使用：** 可配置多種感測器（LIDAR、相機、斷光器等）[pages.nist.gov](https://pages.nist.gov/ARIAC_docs/en/latest/pages/sensors.html#:~:text=Teams%20are%20allowed%20to%20use,execution%2C%20object%20handling%2C%20and%20navigation)；有**成本預算**限制，低於上限有加分，超出扣分[pages.nist.gov](https://pages.nist.gov/ARIAC_docs/en/latest/pages/sensors.html#:~:text=Teams%20are%20given%20a%20budget,found%20on%20the%20scoring%20page)。我們將以最小傳感器集達成任務（檢測 LIDAR、組裝相機、斷光器等），在資訊與成本間取平衡。感測器更新率/解析度也會調整以兼顧成本與效能（如 10 Hz 足矣則不必 30 Hz）[pages.nist.govpages.nist.gov](https://pages.nist.gov/ARIAC_docs/en/latest/pages/sensors.html#:~:text=A)。

**效能與計分：** 計分考量不僅是完成任務，還包括效率與準確性[pages.nist.gov](https://pages.nist.gov/ARIAC_docs/en/latest/pages/scoring.html#:~:text=This%20page%20describes%20the%20scoring,to%20determine%20final%20team%20rankings)[pages.nist.gov](https://pages.nist.gov/ARIAC_docs/en/latest/pages/scoring.html#:~:text=,Human%20Evaluation)[pages.nist.gov](https://pages.nist.gov/ARIAC_docs/en/latest/pages/scoring.html#:~:text=%2A%20%5C%28%5Comega_5%5C%29%20,order%20speed%20bonus%20weight)。重點包含完成訂單數、緊急訂單時效、檢測速度、感測器預算、缺陷辨識成功等；失誤（碰撞、掉落、用到瑕疵件）會扣分。人工作業評估（20%）也計入，將看策略、韌性與設計品質。策略：先確保**穩定與合規**避免扣分，再優化速度拿加分。

**時程：**
- *2025/09/16 – 原始碼釋出：* ARIAC 2025 軟體環境釋出，我們據此開發。
- *2025/12/08 – Smoke Test 提交：* **選擇性** 初階測試，驗證容器能在評測環境建置與執行，不需功能完備[pages.nist.gov](https://pages.nist.gov/ARIAC_docs/en/latest/pages/submission.html#:~:text=The%20smoke%20test%20is%20an,optional%20preliminary%20evaluation%20to)[pages.nist.gov](https://pages.nist.gov/ARIAC_docs/en/latest/pages/submission.html#:~:text=Teams%20are%20not%20expected%20to,your%20code%20runs%20without%20crashing)。
- *2025/12/12 – Smoke Test 結果：* 修復任何相依/設定問題。
- *2026/01/02 – 決賽提交：* **最終代碼** 上傳至私有競賽 GitHub 並打 `final` tag[pages.nist.gov](https://pages.nist.gov/ARIAC_docs/en/latest/pages/submission.html#:~:text=1,git%20tag)。
- *2026/02/02 – 公布結果與頒獎。*

---

# 系統設計與關鍵模組（System Design and Key Modules）

## 感測與知覺（Sensing and Perception）

- **瑕疵檢測（LIDAR）：** 於檢測輸送帶上方以 3D LIDAR 掃描；以點雲擬合理想圓柱並偵測偏差，分類凹痕/刮痕/膨脹[pages.nist.gov](https://pages.nist.gov/ARIAC_docs/en/latest/pages/scenario.html#:~:text=For%20each%20cell%2C%20teams%20must,completely%20cylindrical%20with%20smooth%20surfaces)[pages.nist.gov](https://pages.nist.gov/ARIAC_docs/en/latest/pages/scenario.html#:~:text=Teams%20should%20use%20LIDAR%20sensors,of%20three%20types%20of%20defects)[pages.nist.gov](https://pages.nist.gov/ARIAC_docs/en/latest/pages/scenario.html#:~:text=,beyond%20the%20expected%20cell%20surface)。
- **零件定位（Vision）：** 在組裝區以 RGB-D 取得 **top shell** 隨機姿態，使用模型匹配/標記或 3D 配準定位[pages.nist.gov](https://pages.nist.gov/ARIAC_docs/en/latest/pages/scenario.html#:~:text=The%20next%20step%20is%20to,it%20onto%20the%20partial%20module)。
- **感測器選型與配置：**  
  - 檢測 LIDAR（可能需高等級更新率）  
  - 斷光器（break-beam）作為拾取觸發與輸送帶停擺偵測[pages.nist.gov](https://pages.nist.gov/ARIAC_docs/en/latest/pages/sensors.html#:~:text=Break%20Beam)  
  - 組裝相機（頂視 RGB / RGB-D）  
  - 組裝輸送帶距離感測器（可選）  
  - 換刀對位輔助（可選）  
  控制在預算內、權衡更新率與成本[pages.nist.govpages.nist.gov](https://pages.nist.gov/ARIAC_docs/en/latest/pages/sensors.html#:~:text=A)。
- **資料整合：** 模組化輸出（如 pass/fail 與缺陷類型、top shell 姿態），便於規劃/控制模組消費。

## 操作與控制（Robot Manipulation and Control）

- **運動規劃：** 使用 **MoveIt**（`moveit_py`）控制各機器人，保守路徑確保可靠，後續再優化速度。
- **夾具控制：**  
  - Robotiq 2F-85：**Gripper Command action** 開/關，回饋抓取成功與否。  
  - 真空工具（VG2/VG4）：以 **Attach Tool 服務** 進行耦合/脫離，並控制吸附；監測掉物。
- **工作站與流程：**  
  - 檢測 Robot1（UR3e）：輸送帶取放 → 電壓測試器。  
  - 檢測 Robot2（UR5e）：測試器取出 → 回收或 AGV 托盤。管理 AGV/槽位。  
  - **AGV 移動：** 以 **Move AGV action** 前往 Inspection/Assembly/Shipping/Recycling[pages.nist.gov](https://pages.nist.gov/ARIAC_docs/en/latest/pages/scenario.html#:~:text=There%20are%20four%20possible%20stations,Inspection%2C%20Assembly%2C%20Shipping%2C%20Recycling)[pages.nist.gov](https://pages.nist.gov/ARIAC_docs/en/latest/pages/scenario.html#:~:text=To%20complete%20a%20kitting%20order,at%20the%20recycling%20station%20a)。  
  - 組裝 Robot1（UR5e）：自 AGV 取四顆 cell 放入底殼槽位，確保極性，必要時利用對位孔 re-grasp。  
  - 組裝 Robot2（UR10e+tool）：**換刀模式**（VG2/VG4）、**上殼取放模式**、**翻轉模式**，並與輸送帶/焊接同步[pages.nist.gov](https://pages.nist.gov/ARIAC_docs/en/latest/pages/scenario.html#:~:text=Image%3A%20)。  
  - 龍門焊接：以服務定位焊點並觸發焊接[pages.nist.gov](https://pages.nist.gov/ARIAC_docs/en/latest/pages/scenario.html#:~:text=With%20the%20top%20shell%20in,performed%20through%20a%20service%20call)。
- **座標與校正：** 利用 TF/URDF 基準框架，為 AGV 托盤槽位、工具座等定義相對位姿，集中於設定檔管理。
- **避碰與協作：** MoveIt 載入環境碰撞體；以鎖/狀態機制避免雙臂同時進入同一區域。

## 高階排程與協同（High-Level Scheduling and Coordination）

- **競賽狀態管理：** 監聽 `competition_state`，於 READY 後呼叫 **start competition**；完成或到時可呼叫 **end competition**[pages.nist.gov](https://pages.nist.gov/ARIAC_docs/en/latest/pages/scenario.html#:~:text=1,are%20established%20before%20production%20begins)[pages.nist.gov](https://pages.nist.gov/ARIAC_docs/en/latest/pages/scenario.html#:~:text=call%2C%20transitioning%20the%20state%20to,STARTED)[pages.nist.gov](https://pages.nist.gov/ARIAC_docs/en/latest/pages/scenario.html#:~:text=4,they%20have%20finished%20everything%20required)。
- **訂單/試題處理：** 依試題設定（NUM_KITS、NUM_MODULES）決策流程，並監聽高優先訂單 topic 即時切換[pages.nist.gov](https://pages.nist.gov/ARIAC_docs/en/latest/pages/challenges.html#:~:text=1,reference)。
- **任務序列（FSM/BT）：**  
  - 任務 1 迴圈：控制 feeder、監聽檢測、Robot1 抓取→放測試器、等待電壓→Robot2 放 AGV/回收、AGV 滿格提交與回流。  
  - 任務 2 流程：插入底殼、Robot1 置入四 cell、輸送帶轉移、Robot2 VG2 上殼、焊接四點、換 VG4 翻轉、焊接兩點、提交 module。  
  先以 **有限狀態機（FSM）** 實作，必要時再轉 **行為樹（BT）** 增強反應性[researchgate.net](https://www.researchgate.net/publication/349004128_Team_RuBot's_experiences_and_lessons_from_the_ARIAC#:~:text=,2%5D.)[researchgate.net](https://www.researchgate.net/publication/349004128_Team_RuBot's_experiences_and_lessons_from_the_ARIAC#:~:text=insufficient%20parts%2C%20and%20human%20safety,planning%20with%20the%20quick%20response)。
- **多機協同與並行：** 區域化管理（檢測/組裝各自執行），以共享狀態同步；安全前提下引入並行以提升吞吐。
- **應變處理：** 對應四項敏捷挑戰的暫停/切換/重試策略（前述）。
- **實作語言與記錄：** 以 Python（asyncio 或 multithreading）實現；強化日誌與狀態追蹤便於除錯。

## 對位與姿態（Alignment and Orientation Handling）

- **Cell 極性對齊：** 依殼上「+/–」標示與已知初始姿態，於 Robot1 放置時確保旋向正確；必要時以 re-grasp 完成[pages.nist.gov](https://pages.nist.gov/ARIAC_docs/en/latest/pages/scenario.html#:~:text=be%20used%20to%20perform%20a,grasp%20of%20the%20cell)。
- **換刀對位：** 以預定路徑與姿態靠近工具座並呼叫服務，確保耦合成功[pages.nist.gov](https://pages.nist.gov/ARIAC_docs/en/latest/pages/scenario.html#:~:text=Note)。
- **上殼精準放置：** 依感測姿態緩慢靠近，必要時微調；確保吸盤接觸與對位容差。
- **焊接對位：** 透過輸送帶分段精準定位至焊點，配合服務呼叫完成焊接。
- **AGV 與輸送帶位置確認：** 等待到位回報再作業；必要時以 TF/識別輔助。輸送帶拾取 MVP 可用停帶簡化，後續再嘗試動態拾取。
- **重複測試與校正：** 以設定檔集中管理偏移量，反覆實驗微調。

## 故障偵測與回復（Fault Detection and Recovery）

- **監控 Topics/Services：** 監聽輸送帶/測試器/高優先訂單/夾具回饋等關鍵訊號[pages.nist.gov](https://pages.nist.gov/ARIAC_docs/en/latest/pages/challenges.html#:~:text=Teams%20are%20expected%20to%20handle,this%20challenge%20by)[pages.nist.gov](https://pages.nist.gov/ARIAC_docs/en/latest/pages/challenges.html#:~:text=1,reference)。
- **優雅降級：** 例如單測試器模式、輸送帶暫停時等待、持續失敗則跳過特定模組避免卡住。
- **重試與日誌：** 安全可重試的動作加入 N 次重試與微小位姿調整，並完整記錄。
- **逾時與終止條件：** 防止 deadlock；逾時即切換替代路徑或跳過。
- **人工觸發（開發期）：** 提供手動開關模擬故障以測試回復。
- **挑戰測試：** 為每個挑戰建立試題並驗證回應行為與效能[pages.nist.gov](https://pages.nist.gov/ARIAC_docs/en/latest/pages/config_files.html#:~:text=NUM_MODULES%3A%202)[pages.nist.gov](https://pages.nist.gov/ARIAC_docs/en/latest/pages/config_files.html#:~:text=,TOOL%3A%202%20GRASP_OCCURRENCE%3A%205)[pages.nist.gov](https://pages.nist.gov/ARIAC_docs/en/latest/pages/config_files.html#:~:text=VACUUM_TOOL_MALFUNCTIONS%3A%20,TOOL%3A%202%20GRASP_OCCURRENCE%3A%205)。

---

# 開發計畫（MVP → Core → Enhancement）

## MVP（最小可行）

**目標：** 在模擬中**可靠**完成任務 1 的端到端流程（至少 1 組 kit 並成功提交），作為 smoke test 提交品。

**重點：**
- 環境接線與連通驗證；可啟動/停止競賽。
- 感測簡化版（暫以全部良品或閾值法），先打通流程。
- Kit 建置 Happy Path：Robot1 停帶拾取→放測試器；Robot2 取出→放 AGV；AGV 送 shipping 並提交。
- 先支持 1 組 kit、**無並行**、**動作保守**避免碰撞。
- 完備 Dockerfile 與啟動流程，於容器內可完整執行且不崩潰[pages.nist.gov](https://pages.nist.gov/ARIAC_docs/en/latest/pages/submission.html#:~:text=)。

目標於 11 月底完成 MVP，12/8 前送 smoke test。

## Core（完整功能）

**目標：** 任務 1 與任務 2 **全功能**、**高可靠**，並可處理所有敏捷挑戰。

**重點：**
- 任務 1 全迴圈、兩台測試器**並行**利用、AGV 輪轉；完整瑕疵檢測與檢測報告[pages.nist.gov](https://pages.nist.gov/ARIAC_docs/en/latest/pages/scenario.html#:~:text=After%20the%20inspection%20is%20complete%2C,The%20inspection%20report%20contains)。
- 任務 2 各步驟（底殼、置入四 cell、上殼隨機姿態取放、四上焊、翻轉、二下焊、提交）串聯。
- 極性/對位精化；挑戰處理邏輯整合（輸送帶/測試器/真空/高優先）。
- 多訂單協調（NUM_KITS/NUM_MODULES）與錯誤回復；逐步引入並行提速。
- 參數調校（輸送帶/機器人速度）以滿足時限並爭取效率加分。
- 最終 Docker 影像 `final`，在乾淨環境測試[pages.nist.gov](https://pages.nist.gov/ARIAC_docs/en/latest/pages/submission.html#:~:text=3)[pages.nist.gov](https://pages.nist.gov/ARIAC_docs/en/latest/pages/submission.html#:~:text=1,git%20tag)。

## Enhancement（提升與未來）

- **高層規劃：**（選擇性）PDDL 規劃 + BT 執行，動態重新排程[researchgate.net](https://www.researchgate.net/publication/349004128_Team_RuBot's_experiences_and_lessons_from_the_ARIAC#:~:text=insufficient%20parts%2C%20and%20human%20safety,planning%20with%20the%20quick%20response)。
- **行為樹（BT）：** 提升模組化與反應性；與 FSM 比較。  
- **進階知覺（ML）：** 若需要，提高瑕疵分類準確度（確保極低漏檢）。  
- **最大化並行：** 測試雙測試器流水線、AGV/機械臂與輸送帶重疊調度以提高吞吐[pages.nist.gov](https://pages.nist.gov/ARIAC_docs/en/latest/pages/scoring.html#:~:text=,Human%20Evaluation)。  
- **更強健的回復：** 更豐富的失敗路徑（必要時跨臂協助）。  
- **自動化調參：** 以模擬批次探索最佳速度/節奏。  
- **模組化與效能：** ROS 2 components/Nodelets、必要時熱點轉 C++。  
- **對齊業界最佳實踐：** 參考過往優勝隊策略（PDDL+BT、agent-based 架構）[researchgate.net](https://www.researchgate.net/publication/349004128_Team_RuBot's_experiences_and_lessons_from_the_ARIAC#:~:text=We%20share%20our%20experience%20and,on%20the%20Unified%20Modeling%20Language)。

---

# 程式風格與格式規範（Coding Standards and Formatting Guidelines）

- **Python 3.10+（ROS 2 Jazzy）** 與 **PEP 8**：`snake_case`、4 空白縮排、行長 ~100、模組/類/函式文件字串、關鍵邏輯註解。
- **ROS 2 節點結構：** 使用 `rclpy`，語意化節點與 logger 名稱；功能適度拆分以利並行。
- **檔名與目錄：** 套件/目錄/模組小寫；啟動檔清楚命名（如 `competition_launch.py`）。
- **函式設計：** 短小聚焦，>50 行考慮重構。
- **常數與設定：** 集中在設定檔或 ROS 參數（YAML 載入），如輸送帶速度/逾時。
- **錯誤處理：** 針對性 try-except，記錄上下文，避免裸 `except:`。
- **日誌：** 用 `rclpy.logging` 分層（INFO/WARN/ERROR），例如高優先訂單觸發時明確紀錄。
- **效能：** 高頻 callback 避免重計算；必要時向量化或 C++；以 **profile 後再優化** 為原則。
- **版本控制：** 有意義的提交訊息、功能分支、合併前自我 code review。
- **文件：** README/說明啟動步驟、封裝結構；此計畫文件可作為起點並逐步更新。
- **測試：** 小型腳本/launch 測試；對關鍵演算法做可重現的單元/整合測試。
- **命名範例：** `current_state`、`agv1_goal_station`、`num_defects_detected` 等自解變數名。
- **Launch/Config：** 以引數提升彈性（`use_sim_time`、trial 檔選擇），YAML 加註釋說明各參數。

---

# 開發清單（Development Checklist）

**MVP：**
- [ ] 設定 Docker 並驗證 example 可在本機執行[pages.nist.gov](https://pages.nist.gov/ARIAC_docs/en/latest/pages/startup.html#:~:text=4,Docker%20Compose)  
- [ ] 建立初始 ROS 2 套件與節點骨架：perception / control / coordinator  
- [ ] 實作競賽啟停：等待 READY → 呼叫 start[pages.nist.gov](https://pages.nist.gov/ARIAC_docs/en/latest/pages/scenario.html#:~:text=1,are%20established%20before%20production%20begins)  
- [ ] 簡化瑕疵檢測（暫假設良品）以打通流程  
- [ ] Robot1 自輸送帶（停帶）取一顆放測試器  
- [ ] Robot2 自測試器移至 AGV 托盤（MVP 暫不檢電壓）  
- [ ] 於 shipping 提交 1 組 kit 並驗證通過[pages.nist.gov](https://pages.nist.gov/ARIAC_docs/en/latest/pages/scenario.html#:~:text=To%20complete%20a%20kitting%20order,at%20the%20recycling%20station%20a)  
- [ ] 準備 Dockerfile（smoke_test tag）與 MVP 啟動指令[pages.nist.gov](https://pages.nist.gov/ARIAC_docs/en/latest/pages/submission.html#:~:text=1,image)  
- [ ] 端到端測試：本機容器啟動 MVP，完成 1 組 kit 並無錯誤

**Core：**
- [ ] 完整 LIDAR 瑕疵檢測並填寫檢測報告[pages.nist.gov](https://pages.nist.gov/ARIAC_docs/en/latest/pages/scenario.html#:~:text=After%20the%20inspection%20is%20complete%2C,The%20inspection%20report%20contains)  
- [ ] 整合斷光器/觸發機制，目標在不暫停輸送帶下可靠抓取  
- [ ] 電壓檢查邏輯（±0.2 V），Robot2 僅擺放合格者，否則丟棄[pages.nist.gov](https://pages.nist.gov/ARIAC_docs/en/latest/pages/scenario.html#:~:text=Step%201d%3A%20AGV%20Placement)  
- [ ] 連續處理多顆電池直至完成所需 kits  
- [ ] 兩測試器並行使用，提高產能  
- [ ] 追蹤 AGV 狀態並輪轉（Inspection ↔ Shipping/Assembly）  
- [ ] 檢測雙臂協作同步，避免碰撞  
- [ ] 以服務插入底殼（任務 2 起點）[pages.nist.gov](https://pages.nist.gov/ARIAC_docs/en/latest/pages/scenario.html#:~:text=When%20an%20AGV%20arrives%20at,spawn%20in%20the%20same%20place)  
- [ ] 組裝 Robot1：四顆 cell 正確方向置入底殼[pages.nist.gov](https://pages.nist.gov/ARIAC_docs/en/latest/pages/scenario.html#:~:text=be%20used%20to%20perform%20a,grasp%20of%20the%20cell)  
- [ ] 組裝 Robot2：VG2 換刀與隨機姿態 top shell 取放[pages.nist.gov](https://pages.nist.gov/ARIAC_docs/en/latest/pages/scenario.html#:~:text=The%20next%20step%20is%20to,it%20onto%20the%20partial%20module)  
- [ ] 放置上殼並確認吸附/釋放正確[pages.nist.gov](https://pages.nist.gov/ARIAC_docs/en/latest/pages/scenario.html#:~:text=Image%3A%20)  
- [ ] 移動至焊接區並完成四個上焊[pages.nist.gov](https://pages.nist.gov/ARIAC_docs/en/latest/pages/scenario.html#:~:text=With%20the%20top%20shell%20in,performed%20through%20a%20service%20call)  
- [ ] VG4 換刀與 180° 翻轉（不可掉落）[pages.nist.gov](https://pages.nist.gov/ARIAC_docs/en/latest/pages/scenario.html#:~:text=Note)  
- [ ] 完成兩個下焊並提交模組[pages.nist.gov](https://pages.nist.gov/ARIAC_docs/en/latest/pages/scenario.html#:~:text=The%20final%20step%20involves%20performing,module%20service%20can%20be%20called)  
- [ ] 高層邏輯覆蓋所有訂單（NUM_KITS/NUM_MODULES）[pages.nist.gov](https://pages.nist.gov/ARIAC_docs/en/latest/pages/config_files.html#:~:text=)  
- [ ] 高優先訂單處理：監聽、切換 NiMH、優先完成提交[pages.nist.gov](https://pages.nist.gov/ARIAC_docs/en/latest/pages/challenges.html#:~:text=1,reference)  
- [ ] 輸送帶故障處理：偵測停擺並暫停動作、恢復後繼續[pages.nist.gov](https://pages.nist.gov/ARIAC_docs/en/latest/pages/challenges.html#:~:text=Teams%20are%20expected%20to%20handle,this%20challenge%20by)  
- [ ] 測試器故障處理：偵測無資料，切換至另一台[pages.nist.gov](https://pages.nist.gov/ARIAC_docs/en/latest/pages/challenges.html#:~:text=1,reference)  
- [ ] 真空夾失敗處理：偵測失敗並快速重試[pages.nist.gov](https://pages.nist.gov/ARIAC_docs/en/latest/pages/challenges.html#:~:text=1,reference)  
- [ ] 感測器套件定稿：型號/姿態/成本符合預算[pages.nist.gov](https://pages.nist.gov/ARIAC_docs/en/latest/pages/sensors.html#:~:text=Teams%20are%20given%20a%20budget,found%20on%20the%20scoring%20page)  
- [ ] 參數調校以滿足時間限制並爭取效率加分  
- [ ] 混合情境整合測試（多 kit、含挑戰）  
- [ ] 最終 Docker 影像與 `final` 標記，乾淨環境驗證[pages.nist.gov](https://pages.nist.gov/ARIAC_docs/en/latest/pages/submission.html#:~:text=3)[pages.nist.gov](https://pages.nist.gov/ARIAC_docs/en/latest/pages/submission.html#:~:text=1,git%20tag)

**Upgrade：**
- [ ] 規劃器與/或 BT 強化  
- [ ] 運動規劃與並行最佳化  
- [ ] 瑕疵分類精度提升（傳統/ML）  
- [ ] 感測器成本優化（嘗試移除/降級）  
- [ ] 參數自動化調優  
- [ ] 長時間壓力測試（記憶體/穩定性）  
- [ ] 文件/影片（若需）  
- [ ] Lint/Format（flake8/black）  
- [ ] 最終提交檢查：Dockerfile/Config/Launch/README 等完整

## MVP Smoke-Test Package

本倉庫新增 `ariac_team` ROS 2 套件，整理任務一 MVP 的骨架：
- `ariac_team/nodes/mvp_coordinator.py` 會啟動簡化的階段式 pipeline，串接「啟動競賽 → 輸送帶出料 → Robot1 放置測試儀 → Robot2 放置 AGV → 提交 kit → 結束競賽」。
- `ariac_team/pipeline` 定義抽象 Stage 與共用 Context，讓每個子任務可以獨立開發與測試。
- `ariac_team/config/trials/` 提供 smoke test 及子任務專屬 trial（`mvp_smoke`、`conveyor_to_tester`、`tester_to_agv`、`shipping_submit`），後續可於 ARIAC App 或 `ros2 launch` 單獨載入。

### 設定檔說明（Team Config & Trials）

- `config/ariac_team_config.yaml`
  - **輸送帶參數：** `CONVEYOR_SPEED=0.05`、`CELL_FEED_RATE=0.08`，以保守速度確保 MVP 階段穩定。
  - **感測器配置：**
    - `inspection_breakbeam`：輸送帶入口偵測 Cell 到達。
    - `inspection_rgbd`：輸送帶上方 RGBD，供外觀檢測/定位使用。
    - `tester_rgbd`：測試儀上方 RGBD，協助 Robot2 精準抓取。
    - `agv_station_rgbd`：AGV 上方 RGBD，確認槽位與極性。
    - `shipping_breakbeam`：Shipping 區域入口偵測 AGV 進場。
- `config/trials/*.yaml`
  - `mvp_smoke.yaml`：完整快樂路徑，一個 kit，所有 cell 為良品。
  - `conveyor_to_tester.yaml`：鎖定 Robot1（輸送帶 → 測試儀）的調試情境。
  - `tester_to_agv.yaml`：鎖定 Robot2（測試儀 → AGV）的調試情境。
  - `shipping_submit.yaml`：驗證 AGV 前往 Shipping 並提交 kit 的服務流程。

### 執行方式

1. 建置容器（第一次或更新依賴時）：
   ```bash
   docker build -t ariac_team:mvp .
   ```
2. 啟動 smoke test：
   ```bash
   docker run --rm -it --gpus all -p 5900:5900 ariac_team:mvp bash -lc /usr/local/bin/start_vnc.sh
   # 另一個終端，於容器內：
   ros2 launch ariac_team mvp_smoke.launch.py
   ```
   或使用 `docker/start_mvp.sh` 一鍵啟動（含 VNC 環境）。
> MVP 假設所有電池皆為良品（`TesterBypassStage`），後續可逐步替換 Stage 實作以引入真正的檢測與回收流程。
