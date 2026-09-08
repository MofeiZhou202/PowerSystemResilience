# 市场求解性能与资源对照

## SCED 到 LMP 复用与分解诊断

RATIONALE（实施前）：对同一日、同一边界的独立compact SCED，定价原装配的公共
系数保持不变。由定价规则形成有序行选择R和列选择C，有A_price=R*A_sced*C、
E_price=R_eq*E_sced*C，RHS按相同顺序选取；仅删除原LMP规则本来不装配的储能能量
行/列、新能源/交易优先行及相应短缺列。其余行不得引用删除列。共同系数、行顺序、
列顺序和浮点数逐bit保留，不做数值消元、行缩放或epsilon扰动。按原表达式重新
生成定价区间、冻结状态和定价费用；矩阵值直接复制，不重新求和。
依据原assemble_model的pricing分支及执行契约2.6.6条件定价规则。复杂度
O(nnz+n+m)，额外顺序索引/矩阵空间O(nnz+n+m)，释放SCED时无需新建全套Expr。
预计LMP装配3.3 s降到1.3--2.3 s（减少1--2 s）。原始SCED资源结果先完整生成，
水量/SOC及调度状态不变；派生仅限独立compact、未裁行、无RT/辅助服务且无安全割。
reference仍走原装配；verify独立重建原LMP并逐项检查矩阵、RHS、成本、边界、
列/行语义、完整原表达式与结果索引。模型不满足条件时使用原装配。

验收预先固定：旧scale-final5和新版本使用同一2000/1320机组边界、8调度线程，
冷启动5次（每次新进程）与同会话5次重复运行分别测完整浏览器点击到显示；所有运行
串行且不与构建/诊断并行。报告每次、均值和最大wall，不移动60秒原目标；改动阶段
装配节省至少1秒为分项目标。全原残差<=1e-6，完整水量/SOC/资源/价格与旧版精确
一致；每次双定价完整对偶精确一致。另做verify矩阵和跨进程原/派生完整对偶比较。
分解诊断独立于性能样本，只采集原模型/求解器证据，不放宽精度、删设备或调整定价。
跨运行大模型结构缓存（建议第2项）本轮暂不实施；先按第1项、第3项顺序验证。
“冷启动”仅指新服务器进程首次出清，未清除操作系统文件页缓存；进程启动、边界导入
在点击前完成，不计入点击等待。一个新进程先运行一次，再在相同边界revision、相同
浏览器/服务器会话连续重复五次，另启动四个新进程各运行一次。旧版十次后新版十次，
无同时求解；原有用户服务保留。系统后台进程未停止，样本不是独占裸机性能保证。

首个独立性能样本`lmp-reuse-pilot`：完整浏览器等待58.560963 s，LMP装配0.594592 s，
相对旧样本3.272 s节省约2.68 s，比预期1--2 s更好。按偏差协议重新核对：完整非计时
响应与scale-final5精确一致；派生只进行顺序筛选、冻结和索引重映射，未重新创建全套
Expr和map节点，原装配包含的对象分配/销毁成本高于初始带宽模型。保留原验收阈值，
不凭单次样本宣称稳定一分钟。所有模型变化仍须由独立verify比较确认。

诊断工具`probe_market_structure.cpp`仅只读统计原SCUC松弛及同一dispatch的原LMP。
排除零系数和authored固定列后，对列度数d_j计算sum_j d_j(d_j-1)/2，表示法方程
图边的重数代理，**不是消元填充、实际浮点运算量或耗时**。行统计记录非固定列度数
与跨时点名称；小时储能方向变量的最后索引是小时，不能据此认定精确时间分区。
只形成O(nnz+n+m)计数，不形成稠密AA'；实际填充/排序/迭代证据来自生产Gurobi日志。
不对删除水电/储能的改变模型做原结果等价声明。

### 原模型分解瓶颈诊断

独立pilot生产日志给出以下数据；Factor Ops为Gurobi估计的每次分解算术量，
Barrier总wall包含presolve/排序/迭代，并非纯数值分解秒数。并发双定价日志各一份，
两者时间重叠。现有接口没有暴露逐次数值因子化独占wall，不能以相减伪造该指标。

| LP | presolve后行/列/非零 | AA'非零 | Factor NZ | Factor Ops | 排序/s | 迭代/总wall/s |
|---|---|---:|---:|---:|---:|---:|
| SCUC松弛 | 571049 / 1566777 / 3888366 | 5.579e6 | 1.014e8 | 1.284e11 | 3.59 | 22 / 18.91 |
| 固定整数修复 | 459984 / 1502328 / 3338244 | 4.207e6 | 4.901e7 | 2.414e10 | 2.99 | 44 / 18.02 |
| LMP原LP，每份 | 426327 / 1432201 / 2989816 | 3.829e6 | 1.369e7 | 9.464e8 | 0.45--0.46 | 46 / 9.88--10.09 |

松弛Factor NZ/AA' NZ约18.18，修复11.65，定价3.58；该比率是日志统计口径的
填充指标，不能当精确内存占用。主调度LP的排序/填充和修复44次迭代是下一重点。
原矩阵只读计数（`lmp-reuse-checks/structure.json`）定位到：区域备用行最多537个
非固定系数、一次备用汇总517个；360条水电日电量上下界各最多384个跨时点系数；
80条储能循环行各196个；1320条启动次数及1320条停机次数行各98个；120条负荷响应
日电量行各96个。水库守恒和泄流爬坡跨时点局部行最多12/10个系数。功率p列原SCUC
图边重数代理18385561，flow列3141880。原规模/宽行可解释结构耦合，但代理指标不能
给出“水电占多少秒、储能占多少秒”的因果比例；尚未完成presolve后逐设备归因。

后续实验应先针对跨时段宽行与网络块设计保留全约束的排序/分块策略，先证数学等价，
再测真实Factor NZ/Ops、原单位残差和固定定价完整对偶；不可直接删水电/储能或按日
独立求解带有承接状态的周任务。本轮第3项交付为上述结构/日志诊断，未修改求解器排序。

### 五次首次与重复运行验收

旧版`scale-final5`与新版`lmp-reuse-dev`在完全相同边界/Gurobi8线程下，各自十次
完整浏览器点击到结果渲染，全部按上述固定协议串行完成。单位为秒：

| 版本 / 状态 | 第1次 | 第2次 | 第3次 | 第4次 | 第5次 | 均值 | 最大等待 |
|---|---:|---:|---:|---:|---:|---:|---:|
| 旧版 / 首次 | 61.279448 | 61.160908 | 61.292012 | 61.234520 | 60.932560 | 61.179890 | 61.292012 |
| 旧版 / 同会话重复 | 61.407915 | 61.005542 | 61.058316 | 61.078208 | 61.071794 | 61.124355 | 61.407915 |
| 新版 / 首次 | 58.274881 | 58.122112 | 58.190202 | 58.515226 | 58.154075 | 58.251299 | 58.515226 |
| 新版 / 同会话重复 | 58.406352 | 58.570428 | 58.099384 | 58.248357 | 58.160381 | 58.296980 | 58.570428 |

本机新版十次均低于60秒，最大58.570428秒；这是该固定单日诊断边界的实测验收，
不是任意边界/后台负载下的一分钟硬截止保证。首次平均减少2.928591 s（4.79%），
重复平均减少2.827375 s（4.63%）；LMP装配首次平均3.365804→0.590407 s，重复
3.336026→0.594450 s，分别节省2.775397/2.741576 s，超过预定至少1秒门槛。
预测1--2秒与实测差异的重推导见上文对象分配成本说明。额外verify不进入性能样本。
20份完整输入/输出、阶段计时、服务器日志、桌面/移动截图、实际命令位于
`output/market-performance/lmp-reuse-repetition/`。冷启动/重复完整报告为`summary.json`。
`verified-summary.json`逐份确认全部非计时响应与scale-final5精确一致，包括所有资源、
水位/水量/SOC、目标、残差和节点价格。独立verify匹配六个矩阵及全部元数据；SCUC/SCED
最大原单位残差2.1792512e-10、LMP2.3235316e-9。跨进程原/派生定价3247054行对偶
逐bit相同，SHA256为8ce718243b40c07274a276dc6e7dce446dcb7f044d33ee31fff667803c0cc31a，
注入失败时全部价格为空。没有改变价格代表策略或接受近似对偶。

最终Release74测试/29807断言通过，涵盖七日小模型reference/verify资源、节点价格、
water/SOC/state/carry逐日一致及残差，另覆盖交易费用、三类启动状态、非零储能最小
功率、不同定价策略及fallback。聚焦Gurobi的ASan/UBSan7测试/377断言通过，
detect_leaks=0且未关闭容器检查；部分Release归档沿用，不宣称全库sanitizer验证。
两组Southern/operation GUI/API回归通过。记录见`lmp-reuse-suite-final.log`、
`lmp-reuse-asan/tests-final.log`及两份`lmp-reuse-*-e2e.log`。

新版入口<http://127.0.0.1:8107/xjtu/#southern-market>保存相同2000节点边界且启动时
未运行，原8097及8101--8106保留。`lmp-reuse-dev/run_gui_server` SHA256为
948626bcdc7602d27352f3baf333d4a6599a23bcc81e6d2707f18f462c20317a；本机M4 Max/128GiB、
macOS arm64、Release -O3。`build.json`/`test-rebuild.json`/`verification.json`记录
增量编译与归档/源码哈希；未修改依赖准入。2000节点完整七日及544火电版本仍未实测。

```bash
python3 tools/market_validation/benchmark_market_repetition.py --baseline output/market-performance/scale-final5/run_gui_server --candidate output/market-performance/lmp-reuse-dev/run_gui_server --input output/market-performance/scale-final5-browser/boundary.json --output output/market-performance/lmp-reuse-repetition
node tools/market_validation/profile_market_scale.mjs --server output/market-performance/lmp-reuse-dev/run_gui_server --input output/market-performance/scale-final5-browser/boundary.json --threads 8 --assembly verify --output output/market-performance/lmp-reuse-verify
output/market-performance/lmp-reuse-checks/probe_market_scale_price output/market-performance/scale-final2-browser/boundary.json output/market-performance/scale-price-probe/snapshot.json output/market-performance/lmp-reuse-checks/derive.json derive
output/market-performance/lmp-reuse-checks/probe_market_structure output/market-performance/scale-final2-browser/boundary.json output/market-performance/scale-price-probe/snapshot.json output/market-performance/lmp-reuse-checks/structure.json
```

## 2000 节点一分钟目标

基准固定为 GUI 默认 ACTIVSg2000 水电主导配置：2000 母线、3206 支路、120 火电、
720 水电、480 风光、180 水库、80 储能、98 时点。单次定义先按完整日 SCUC/SCED/LMP
及价格独立复核，基线Gurobi4线程、最终8线程、请求gap1%、schedule_only，非交流安全认证。
`profile_market_scale.mjs` 使用隔离服务器、保存完整输入/输出与内部日志，不修改现有会话。
price-final 基线187.889 s以SCUC TimeLimit失败；2571444列、13840整数、10546746非零，
装配4.715 s，未成功预热19.044 s。根障碍法19.83 s，crossover112.74 s，最后无incumbent。
数据在 `output/market-performance/scale-baseline/`；失败不能算一分钟内完成。

### 上一阶段验证与使用入口

最终`scale-final5`同一输入、隔离服务、逐次独立浏览器点击到结果显示：
**60.345731 s / 59.457932 s，均值59.901832 s**。两次均有效出清/价格有效；严格
“每次<=60 s”未稳定达到，不能只选较快一次作为达标证明。测量时没有并行编译、测试或
其他本次任务求解。当前主要成本仍是SCUC约38 s、完整双定价约11 s，装配约6.4 s；
120台火电版之外的544台火电/1744机组版本、2000节点完整七日及AC安全迭代未在本轮验证。

完整模型保留2571444列、13840整数、939606等式、2409528不等式、10546746非零项。
SCUC/SCED目标1451667554.413847元，独立下界1450719989.5625193元，实际gap
0.0652742323%，最大原单位残差2.1792512e-10；LMP目标1723129284.8238487元，
残差2.3235316e-9。两次结果及`verify`与scale-final2的**完整非计时响应精确一致**，
包括目标、残差、设备出力、所有水量/SOC序列、节点价格及边界快照。verify额外原路径
重建的六个矩阵精确一致；该正确性运行与其他测试重叠，不计为性能样本。
本轮不把新的gap内候选与优化前未成功的原MILP称为同一最优解，也不宣称水电/储能有误。

跨进程固定同一份定价LP的3247054个原单位行对偶逐bit相同，完整向量SHA256
`8ce718243b40c07274a276dc6e7dce446dcb7f044d33ee31fff667803c0cc31a`。
两进程均通过生产内部双重核验；注入失败检查时全部节点价格为空。证据为
`scale-price-probe/final5-{result-1.json,result-2.json,comparison.json}`及两份完整二进制对偶。

最终Release **71测试/29589断言**通过；单日和运行模拟GUI/API回归均通过，覆盖编辑、
原子校验、revision冲突、保存/重载、基准/恢复、周月任务和移动端。ASan/UBSan的
Gurobi新路径5测试/327断言通过，`detect_leaks=0`且未关闭容器检查。另一个一致的
sanitizer归档构建在scale-final2源码阶段通过6测试/177断言、1项因无Gurobi跳过。
混合Release/ASan归档的扩展HiGHS测试曾在其未instrument的容器访问处失败，原日志保留
`scale-asan/{tests,coherent-tests}.log`；不能把聚焦Gurobi通过写成全库sanitizer通过。

JSON所有权优化的分项验证：两次最终非阶段开销约1.90 s，比scale-final4约2.25 s减少
约0.35 s，接近预测0.5 s；SCUC运行波动约0.6 s，足以使总等待跨越60 s。索引、输出
优化原预测与失败重推导完整保留在下文。下一性能余量需要继续降低求解器主体成本；
当前不能提供严格的一分钟硬截止保证。

运行入口：<http://127.0.0.1:8106/xjtu/#southern-market>，PID99763，保存上述2000边界，
启动时未运行；原8097、8101--8105会话保留。旧服务不会因源码改变自动更新。
服务器`scale-final5/run_gui_server` SHA256：
`55ca6063e2c633e172f0841666f2a22fecfa60d40dbb836836db7bdd985b0285`。
本机Apple M4 Max/128GiB/macOS arm64，Release -O3；这是保留既有归档的本地增量
实验，非干净发布构建。`scale-final5/build.json`逐级引用父记录并保存实际编译/链接命令；
`scale-final5/verification.json`补充源码/头/工具/依赖/机器与测试证据，未放宽依赖保护。

```bash
node tools/market_validation/profile_market_scale.mjs --server output/market-performance/scale-final5/run_gui_server --input output/market-performance/scale-baseline/boundary.json --threads 8 --browser --output output/market-performance/scale-final5-browser
node tools/market_validation/profile_market_scale.mjs --server output/market-performance/scale-final5/run_gui_server --input output/market-performance/scale-baseline/boundary.json --threads 8 --browser --output output/market-performance/scale-final5-browser-repeat
node tools/market_validation/profile_market_scale.mjs --server output/market-performance/scale-final5/run_gui_server --input output/market-performance/scale-baseline/boundary.json --threads 8 --assembly verify --output output/market-performance/scale-final5-verify
node tools/market_validation/compare_market_scale.mjs output/market-performance/scale-final2-browser output/market-performance/scale-final5-browser output/market-performance/scale-final5-browser-repeat output/market-performance/scale-final5-verify
```

性能脚本先查询服务schema；旧二进制不支持大型策略时只省略隐含auto，显式不支持的
策略仍拒绝，避免给历史版本注入未知字段。截图和原始完整结果保存在各运行目录。

### 推导与实验记录

RATIONALE（网络界限实验，实施前）：对每个可用支路正电纳且零移相的连通分量，
令节点净注入盒上界为 U_i，P=sum_i max(0,U_i)。在任意相角阈值割上，所有支路
潮流均从高角流向低角，因此每条支路 |f_e|<=P（离散最大值原理/割集功率守恒）。
由 f_e=b_e(theta_i-theta_j) 和参考角0，任意参考点路径给出
|theta_i|<=P*sum_path(1/b_e)；采用最短正权路径收紧。逐次向外舍入避免下舍入收紧。
可用支路含非正电纳/非零移相时该时点拒绝应用，保留原式。无拓扑删减、无时间聚合、
无机组删减；这些界限对原可行域有效。SCUC/SCED采用可选认证界限，定价先保留原LP。
成本为98次 O((B+L)log B + nnz(injection))，新增O(B+L)临时空间；M4 Max/128GiB。
预测网络510188列由自由变为有限界，根crossover至少减少60%，SCUC总wall至少减少40%。
该分项不是60秒达标承诺；完整单日wall<=60秒才满足用户目标。条件/公式对应代码
`certified_network_bounds`，数值准入仍为全部原约束残差<=1e-6、价格复核精确一致。
小例原/收紧模型目标差<=1e-6，涵盖断线孤岛、零注入、负电抗/移相拒绝与稳定ID。
定量验证使用同一保存boundary、--network-bounds certified/none；原日志、目标、gap、
残差和完整wall均保留，不以到时/缺价结果宣称完成。

界限实验未达到预测：原障碍法39迭代/19.83 s，有限界限后181迭代/85.85 s且次优退出，
最终180秒仍无整数解。重新推导按顺序检查：小例原/收紧目标等价（前两例26断言通过），
负电抗输入原本被schema拒绝，修正该测试预期；大例确实消除了free变量，但矩阵因子
运算从1.015e10增加至1.118e10，初始目标尺度从1.94e13增至5.14e15，过宽界限损害
数值条件。失败属于成本/条件假设，不是删改水电后得到的提速。该实验不启用为生产默认，
源码试验补丁及原日志在scale-bounded/。直接dual_simplex也在186.994 s以SCUC失败。

下一项 RATIONALE（LP界与可行整数修复，实施前）：标准分支定界有L<=z*<=U，L来自
原MILP连续松弛的最优LP求解器ObjBound，U来自固定候选整数后的原约束可行解。
若(U-L)/abs(U)不超过请求gap，则不需要继续构造根单纯形基/整数搜索来满足同一gap。
只在最优LP状态、有限solver bound和全原单位残差/整数审计通过时准入；失败走完整MILP。
这仍是数值求解器容差内的界，不是精确有理数证明。候选整数取松弛解四舍五入，修复只
固定整数并重解全连续模型；原可行域/成本不改变。已有根日志仅剩3个分数整数支持该假设。
参考：标准MILP LP-relaxation bound与Gurobi ObjBound/Method/Crossover属性说明。
成本两次稀疏LP（预测20+15 s）替代>180 s无解的根搜索，SCUC预测<=45 s；全部原始
约束/整数残差<=1e-6，报告实际gap与bound，不把近似称为精确最优。首先用独立诊断验证
候选可行性和耗时，再决定是否进入生产；小例穷举/原MILP对照验证界和gap。

首个独立修复诊断：连续LP45.832 s（预估20 s未达到），修复23.569 s，候选目标
1451648809.132495，原残差3.11729e-7。纯障碍法的内部点有2214个分数整数；此前3个
指转基后的顶点，不能把二者混同。连续LP因不含MIP预处理，因子规模更大，造成成本预测
偏差。ObjBound返回-1e100，虽调用成功也是不可用哨兵，诊断拒绝准入，未称为有效gap。

修正 RATIONALE：用拉格朗日盒下界独立核验任意有限行乘子。对Ax<=b、Ex=d，取
y<=0、lambda自由，r=c-A'y-E'lambda，则
L=y'b+lambda'd+sum_j min(r_j*l_j,r_j*u_j)<=z*。
对偶符号投影与每次乘加向外舍入给出保守下界；不依赖Gurobi ObjBound或松弛原始可行性。
只在核验时使用网络最大值原理界，不传给求解器，避免此前条件恶化。surplus由分量
总正注入上界控制；非负成本的线路/断面松弛存在互斥的最小超限代表，其界由有界flow
和 authored limits给出，故收紧的盒仍包含至少一个原模型最优解。未知自由变量、移相或
非正电纳时不生成证书。原始整数候选必须另行满足全部1e-6原约束审计。
复杂度 O(nnz+n+m)，预计核验<=2 s、下界与修复方案相对gap<=请求1%；对偶任意扰动
仍须保持弱对偶不等式，未覆盖形态拒绝。松弛LP允许较早结束（BarConvTol<=gap/2），
因为准入由独立下界决定；修复与定价仍保留原精度。小例弱对偶/精确MILP与大例门槛不变。

### 2000 节点后续准入

family hint版scale-final4两次完整浏览器59.762/60.171 s，第一轮达到60 s，但重复超过，
不能宣称稳定达标。两次装配合计6.220/6.342 s，相对scale-final2的6.636 s减少约0.4 s，
接近0.5 s预估；SCED映射导出近零，全部原输出精确一致。
下一输出所有权 RATIONALE：finish lambda返回捕获的out不会获得局部返回值优化，
形成整份结果深拷贝；HTTP另把结果复制给southern_latest。两处原对象随后不再读取，
可用move保持全部JSON字段/数值，只改变所有权。序列化和move在同一会话锁内，
保留revision/stale判定原子性；busy在保存后释放。实体结果行最后插入也使用move。
数据传输仍约148 MB，不删向量或响应字段；成本从两次O(JSON节点数)复制变为O(1)
所有权转移，预测完整wall再减少至少0.5 s，原全输出/矩阵/GUI契约和<=60 s门槛不变。

批量索引实验scale-final3实测61.594 s。SCED导出0.00000025 s，达到0.55 s节省预测；
节点导出audit合计又减少约0.1 s。但排序装配从3.167/3.469增至3.661/3.966 s，
方向与预测相反。独立原路径矩阵/所有非计时字段精确一致，排除数学实现差异；
排序对象持有string_view，比较仍需访问离散字符串，额外排序/临时数组成本未被模型计入。
移除批量排序。修正 RATIONALE：按变量family保存最后一次map插入位置，对同一家族
相邻时点/设备尝试emplace_hint(next(previous))；数字字符串在9/10等处跳序时，
std::map按标准比较自动退回常规查找，键序/重复键的首项语义不变。最坏O(n log n)，
连续词法区段的树定位摊销O(1)，额外空间O(变量家族数)，不增加O(n)排序数组。
预测相对scale-final2两次装配合计至少减少0.5 s，结合已证实的0.55 s导出及节点查找
减少，完整浏览器目标仍<=60 s；reference继续原插入，要求所有矩阵/非计时输出精确一致。

12线程实测61.481 s，SCUC37.673 s，仅比8线程减少0.494 s，定价增至11.594 s。
未达到预估3.8 s；模型和审计均正确，成本假设过于乐观，线程间同步/内存成本抵消收益。
候选目标变为1451691505.497696、gap0.0799%、残差1.03698e-8；不将不同候选称为完全相同。
保留8线程。下一纯装配/导出优化 RATIONALE：原有序列名索引逐项树插入存在O(n log n)
指针追踪。先在连续数组排序(name,column)，再有序hint插入，复杂度仍O(n log n)，
但比较转为连续数组访问、树构造为O(n)；保留最终map键序、列序和值，reference不变。
预计两次装配合计减少约1.5 s。SCED确实复用原解且使用派生模型时，直接移交其已存的
原解映射（O(1) move）替代同一2.57M条映射的重建，预期再减少0.55 s。
节点导出复用已算的平衡行索引，优化路径按既有column_slots读缺额/富余，避免重复树查找，
预期三阶段合计减少0.2 s。验证同8线程scale-final2全部非计时输出精确一致、verify原路径
矩阵精确一致、全市场测试和sanitizer；浏览器<=60 s门槛保持不变。

scale-final2 的浏览器实测61.394 s，相比63.626 s减少2.232 s。三阶段audit
降至0.600/0.597/0.611 s，solution_map合计1.125 s；audit约减少0.45 s/阶段，
接近预估0.5 s，导出减少约0.4 s。原残差不变，仍未达到60 s。
下一参数实验 RATIONALE：M4 Max有12个性能核，固定8线程可能限制稀疏障碍法因子
并行；Amdahl模型假设约30%的阶段时间受串行预处理/内存限制，8到12线程理想上限
约23%提速，保守预测SCUC38.2 s减少至少10%（3.8 s）。只改调度threads=12，
定价保持固定8线程和完整双对偶核验，原模型/精度/gap不变；完整浏览器<=60 s，
残差<=1e-6、独立下界gap<=1%、同一定价LP全对偶精确相同为原验收门槛。
线程数可改变gap内候选，结果差异必须报告。参考Gurobi Threads参数与Amdahl定律；
命令为profile_market_scale.mjs同一boundary、scale-final2二进制、--threads 12 --browser。

恢复严格设置后的scale-final浏览器实测63.626 s，SCUC38.594 s、SCED装配0.198 s、
定价11.099 s，原最大残差2.32354e-9，gap0.0653%，全对偶精确一致。尚差约3.6 s。
最后输出开销优化：结果时间序列沿用Build已有family/id/time列槽，免除每个值两次
O(log n)字符串树查找；reference保留旧查找。solution_map按已有有序列名使用end hint，
构造由O(n log n)降至O(n)。不改变遍历/浮点顺序、输出字段或任何模型。预测每阶段
结果生成至少减少0.5 s，两个solution_map合计减少0.5 s；原结果向量/目标/残差/价格
逐项完全一致，浏览器总wall仍按原60 s门槛评价。

派生SCED实测65.750 s，SCED装配0.194 s/核验0.039 s，原残差2.95131e-10，
gap0.0926%，完整定价对偶一致。派生装配<=1 s预测通过。较松的候选精度改变了候选
轨迹，定价从约11.6 s增至14.1 s，抵消SCUC减少；没有稳定全链收益。最终恢复
relaxation BarConvTol=gap/2（上限1e-2）和修复原默认精度，保留派生SCED。
一分钟目标仍未闭环，最终二进制/浏览器复测另列，不把65.75 s写成<=60 s。

空窗口实测69.024 s，三阶段装配3.456/3.696/3.640 s。优于原5 s但没有达到<=3 s。
剩余装配需真实创建变量/行并统计，不能由空窗口优化消除。最终小范围派生条件：
无实时/辅助服务、row_presolve=none、全部机组startup/minimum成本0、调频预留0、
SCUC/SCED交易费相等时，源码中uc分支只剩固定u/start/stop/stable/offline/start类别。
在原LP上固定这些列，清理整数索引并重算界限审计，保留全部矩阵/RHS/成本；逐列检查
新界限为原界限子集。SCED随后仍与SCUC快照逐项比较，verify模式额外独立原式装配
compare_assembly。条件不满足走常规装配，reference模式也保持常规装配。预期SCED
装配<=1 s，比3.7 s减少>=2.7 s，时间/存储复杂度O(n+nnz)且避免生成全部Expr行。

修复LP允许BarConvTol=1e-6，松弛阈值从gap/2改为gap（上限仍1e-2）。这仅改变候选
生成精度；原矩阵所有约束/整数/水位残差1e-6与原MILP独立gap门槛不变，失败按原预算
回完整MILP。预测两项合计再减少约3 s；不以LP的Optimal标签绕过原单位检查。

PreDual1反例：因子运算升至8.190e12，54.29 s只完成5次迭代，松弛TimeLimit，随后按设计
回退原MILP。该隔离诊断在93 s仍未完成时终止，日志保留scale-dualized-run/server.log；
没有有效出清、不可计为成功等待时间。原图分隔器假设不适用于对偶化后的消元图，移除
该强制参数。没有凭该失败放宽任何数值门槛。

装配空窗口优化 RATIONALE：2.6.3.12仅在minute(t)-minute(s)<min_up/down时添加约束。
时间严格递增且s<=t，故min_up=min_down=0时集合为空。当前1320机组全部为0，却仍执行
G*T*(T+1)/2次扫描；提前判空把这些约600万次/阶段扫描降为O(G)，非零窗口和原顺序
完全保留。只缓存每机组两个标量，预测装配5 s降到<=3 s，三阶段合计减少>=6 s。
reference装配保留旧扫描用于逐项矩阵/成本/RHS/所有恢复结果对照；原门槛为精确相同。

显式BarOrder1与自动模式的松弛因子和逐迭代数值完全相同：自动模式已选择该排序，
故该开关不形成优化，应移除冗余强制配置。下一等价实验采用PreDual1，在Gurobi内部
对LP做对偶化及postsolve，仍导出原行序Pi；Box下界独立用原LP重新计算。
依据线性规划强对偶与Gurobi PreDual参数契约。当前列数约为预处理后行数的2.7倍，
交换原对偶形式可能减少正规方程的填充；这是条件性预测，原行/列数不足以保证收益。
预测松弛/修复因子运算至少减少50%、累计<=26 s，完整<=60 s，原残差和全对偶门槛不变。

负荷候选实测72.551 s（scale-load-run），修复18.778 s，因子2.414e10仍远大于
预测，原残差2.17926e-10、gap0.0653%。两类跨时段资源都固定后没有形成预计的低填充
图，说明仅按设备家族判断因子复杂度的假设不成立；不得将少量提速归结为“水电或储能
模型错误”。下一实验保持LP数值与约束，选Gurobi BarOrder=1（nested dissection），
代替自动稀疏Cholesky排序。参考Gurobi BarOrder定义、George/Liu的稀疏消元图分隔与
填充分析：对时空网络图，分隔器排序可减少消元填充，实际取决于预处理后的图。
成本O(nnz(L))存储及因子运算，预测relax+repair因子运算减少至少50%、累计求解<=26 s，
总wall<=60 s；若不满足，保留原自动排序并记录失败。原可行性/下界/全对偶检查不变。

储能投影实测失败（scale-storage-run）：投影0.079 s，修复20.952 s未达到<=12 s，
因子运算2.491e10仅降至2.464e10，完整75.085 s；审计1.56降至约1.09 s，接近0.5 s预测。
原因按假设重查：投影实施正确且全原残差3.97637e-9，原模型无删减；仍有120个可控负荷
全天能量上限连接各时点。故不能归因“储能是主要瓶颈”。下一候选同时固定可控负荷：
将松弛削减量裁入[0,available*max_reduction]，逐母线按比例缩至load_mw，再逐负荷按
比例缩至max_day_reduction_mwh。所有缩放只减小非负削减量，保持前面已满足的上界。
该轨迹是可行初解候选，原LP上下界/费用和完整下界不变；原全约束残差与gap仍决定准入。
复杂度O(D*T)，预测附加<0.1 s，修复因子运算至少减少70%，修复<=12 s；反证仍原样保留。

首个生产API实测78.051 s（scale-certified-run，12调度线程），SCUC41.576 s、
SCED核验0.048 s、并发定价12.848 s；完整3247054行对偶精确一致，残差5.17685e-9。
并发定价<=16 s预测通过，但全链仍未达到60 s。整数修复的因子运算约2.5e10，
定价约8.8e8；后者保留所选原定价规则，不能直接替代调度能量模型。

下一诊断 RATIONALE：从原LP抽取全部储能方向/能量/循环行，固定已选择整数方向，
解 min sum_t (|dis_t-dis_relax_t|+|ch_t-ch_relax_t|) 的小LP。其结果只作为全网整数
修复的候选储能功率；所有原全网约束和费用保留，只有候选LP额外固定这些功率。
原MILP下界仍由未固定储能的完整松弛产生，因此最终通过原可行性与1%gap才能采用。
抽取失败或候选质量不足回原MILP。算法是L1投影启发式，不是等价模型消元，也不保证
储能轨迹与此前另一个gap内解相同。原水量/SOC方程和跨日承接口径必须全部保留。
预期O(S*T)小LP<1 s，固定其跨日连接后全网修复<=12 s，完整wall至少减少8 s；
剩余差距还需实测。仅固定最小充放电功率为零的普通储能，排除实时/辅助服务。
同时将结果统计的逐行JSON查找改为原顺序标量累加，最后序列化，预期每阶段audit减少
至少0.5 s；原输出字段、浮点累加顺序、矩阵、目标与残差须与原统计逐项相同。

8 线程、BarConvTol=.005 的松弛 LP19.979 s，整数修复19.706 s，目标1451648809.132451，
独立盒下界1450719989.562520，gap0.000639838，原残差1.06582e-10。SCED逐项矩阵/
成本/RHS相等，界限为原界限子集，候选仍满足同一残差。这允许由下界继承与可行性复核
替代SCED重解，但必须逐项检查，不能只比较算例ID或尺寸。还要求无辅助服务、常数成本
一致、列语义顺序一致，SCED整数为原整数集合子集；否则走原求解路径。

修正成本预测：盒构造4.807 s超过预计2 s，主要成本为表达式拷贝和逐非零项map查询。
改为原行引用和列索引数组，将证书构造与独立固定整数LP并行（原模型只读），预测其wall
不再叠加于19.7 s修复。保留局部原LP快照用于SCED精确比较，额外O(nnz+n+m)内存/复制。
失败实验的网络界限schema已移除，界限不再改变求解器模型。

定价预试验专用障碍法8线程/Crossover0两次各10.986/10.850 s，完整原约束残差均
9.27372e-10；尚需生产完整双对偶精确比较。下一实施使用显式版本
`ordered-lp-barrier-8-v2`，只用于Gurobi且列数>=1000000的定价LP，固定Method2/Threads8/
Seed0/Crossover0，重置环境覆盖；小LP保持v1。两次独立环境并发，只读同一LP，分别
收集计时、恢复原单位对偶，原有精确相等门槛保持不变。数学LP不变，但退化对偶代表政策
变更，不能承诺与旧v1价格相同。参考Gurobi Method/Threads/Seed/Crossover文档。
成本两次O(nnz)导入与障碍求解，预测并发wall<=16 s（顺序21.8 s），内存翻倍。最终
端到端目标仍为60 s；若未达标保留实测差距，不以分项时间替代HTTP总等待。
验证：原LP审计<=1e-6；SCED矩阵/目标/RHS逐项相等；全行对偶差0，跨进程复算亦相同；
小例下界/回退/价格门控；全市场回归；同一2000boundary的API完整wall。

## 定价一致性闭环

同一最终二进制的四组完整七日浏览器等待时间：

| 原因分析 / 恢复定价 / 装配 | 实测 s |
|---|---:|
| always / full / cached | 170.628 |
| always / dispatch_only / cached | 140.670 |
| anomaly / dispatch_only / cached | 41.771 |
| anomaly / dispatch_only / verify | 47.473 |

四组顺序运行，计时期间不并行跑测试或构建。所有运行的价格检查通过；严格 comparator
现在 exit 0：主价格最大差 0，147 个对照阶段目标差 0、残差差 0，主资源、水量/SOC、
线路与恢复指标精确一致。verify 主链 21 阶段/42 矩阵精确一致。
见 `price-week-auto/comparison.json` 和各 price-week-*/job.json/report.json；命令汇总为
`price-week-commands.json`。原比较器的价格容差没有改变，旧失败产物仍保留。

预测默认周新增约3.5 s，实测较前版38.562 s增加3.209 s（+8.32%），满足 <=50 s 门槛；
没有触发预测偏差超过50%的重新推导条件。默认周复算 wall 累计3.988 s，端到端增量
还包含主 LP 算法切换和运行波动；不能直接把两者当成同一统计量。
全恢复49个定价 LP 的复算 wall 累计28.921 s，恢复的并行部分不能与浏览器 wall 相加。
相同新版本中默认周比完整恢复少75.52%等待，但本例无供需/越限异常，不能推广到异常周。

与旧自动并发定价策略单独比较：物理轨迹和水量/SOC精确一致，147阶段最大目标差
4.47035e-8、残差差1.26617e-9，均低于1e-6。旧价格到新固定代表的最大变化为
10.6907241 CNY/MWh；这是定价选取政策变更，不能声称新价格与旧历史价格完全相同。
见 `price-week-full/old-policy-comparison.json`。新版本各模式间价格差严格为零。

新服务 `http://127.0.0.1:8105/xjtu/#market-operation`，PID66948，原IEEE118边界/seed，
anomaly/dispatch_only，启动 ready 0/7；原8097/8101/8102/8103/8104保留。
桌面/390px移动端价格检查表和结果页已实览，新表沿用有界横向滚动，无新增文档宽度溢出。
启动记录与截图在 price-service/，实际结果截图在 price-week-auto/。

最终增量二进制位于 `output/market-performance/price-final/`，server SHA256
`87becb70f7da09e3a5a6170d8ad3412fe3febf6e51673a08c4462134a8643442`。
沿用下文 Apple M4 Max/128 GiB、arm64 Release -O3、两仓库 HEAD 和已有依赖归档；
并非干净发布构建，依赖保护未修改。完整构建命令、源码/归档/二进制哈希见 build.json，
新内部检查头和测试哈希另见 supplementary-source-hashes.json。
Release 68 测试/29305 断言、最终二进制 operation/forecast GUI/API 回归通过；
ASan/UBSan 价格专项 2 测试/26 断言通过，无 Gurobi 链接、detect_leaks=0，
增量重编译记录见 price-asan/rebuild-evidence.json。未执行 MIPSolvers 独立全库测试。
首次新增测试误传 HiGHS 非零 threads 被已有契约拒绝，修正测试后重新运行全套。

保存的价差反例用同一装配 LP 验证：两个新进程内分别请求 auto/dual_simplex/barrier，
199246 个原单位行对偶全部精确一致；目标 14034457.13961928，最大残差
1.1596057447604835e-10。每个请求还独立复算，故共检查 12 次 LP 求解。
注入失败检查后，生产 stage_result 导出 prices_valid=false、全部节点价格 null。
见 `price-probe/comparison.json`、results-1/2.json 和 build.json；测试驱动为
`tools/market_validation/probe_market_lmp_duals.cpp`，其 include-source 诊断链接替代
southern_market 对象，不扩展生产 API，也不在生产路径注入测试开关。

本轮针对下文保留的 6.64876 CNY/MWh 价差失败证据，采用
`ordered-lp-dual-simplex-v1`。数学、接口和失败处理见执行契约的
Deterministic Pricing。此前自动并发算法价差是历史失败，不改写为已通过。

RATIONALE（实施前约定）：对象为固定 SCED 离散状态后的原定价 LP；退化最优面
有多个对偶，固定单线程对偶单纯形消除自动并发算法的胜者竞争。模型、稀疏行列顺序、
目标系数、权重和承接保持不变。两次 fresh solve 后比较全部原单位行对偶，代价为
额外一次 LP 导入/优化/提取和 O(m+n+nnz) 审核、O(m+n) 解向量存储，无历史模型缓存。
假设相同后端版本、平台、行列顺序及配置，均在预算内达到最优；不承诺跨后端唯一对偶。
依据为 Gurobi Method/Threads/Seed 与 HiGHS simplex_strategy/parallel/random_seed
参数契约，以及节点 LMP 的原对偶恢复公式。预测七日默认模式较 38.562 s 增加约
3.5 s；要求 <=50 s。数值准入：同 LP 全部对偶与节点价格差严格为 0；目标与原单位
残差对照 <=1e-6，水量/SOC/调度精确相同；不放宽既有 comparator。

验证命令（各模式顺序运行，性能测量期间无其他代理计算）：

```bash
python3 tools/market_validation/build_market_overlay.py --output output/market-performance/price-final
output/market-performance/price-final/test_southern_market
node tools/market_validation/replay_market_forecast.mjs --server output/market-performance/price-final/run_gui_server --assembly cached --trigger always --pricing full --output output/market-performance/price-week-full
node tools/market_validation/replay_market_forecast.mjs --server output/market-performance/price-final/run_gui_server --assembly cached --trigger always --pricing dispatch_only --output output/market-performance/price-week-dispatch
node tools/market_validation/replay_market_forecast.mjs --server output/market-performance/price-final/run_gui_server --assembly cached --trigger anomaly --pricing dispatch_only --output output/market-performance/price-week-auto
node tools/market_validation/replay_market_forecast.mjs --server output/market-performance/price-final/run_gui_server --assembly verify --trigger anomaly --pricing dispatch_only --output output/market-performance/price-week-verify
node tools/market_validation/compare_market_recovery.mjs output/market-performance/price-week-full output/market-performance/price-week-dispatch output/market-performance/price-week-auto output/market-performance/price-week-verify
```

本页记录 IEEE118 合成水火风光储边界的性能实验。设备删减对照改变可行域，只用于诊断，
不得把它们的运行时间或目标值当成等价优化结果。真实生产变更必须保留水量、SOC、
机组组合、网络、备用、定价及既有求解质量门槛。

## 资源对照协议

`tools/market_validation/profile_market_resources.mjs` 启动独立服务，不修改用户现有会话。
使用 `ieee118_mixed`、98 点、诊断罚价 100000、MIP GAP 0.01、单次求解时限 15 s；
Gurobi 固定 2 线程，HiGHS 保留既有线程行为。每个求解器进行两轮，第二轮反转消融顺序。
对照为完整模型、仅移除水库约束、仅移除储能、固定基准水电组合、固定基准火电组合。
固定组合来自本轮已验证可行的基准解，不是把所有机组强制开机。

命令：`node tools/market_validation/profile_market_resources.mjs --repetitions 2 --seconds 15`。
原始记录及边界在 `output/market-performance/resources/`，含二进制 SHA256、源码提交和机器型号。
该实验的各差值不能相加：整数搜索、预处理和退化均会产生交互。

| 求解器 | 完整单日 / s | 无水量约束 / s | 无储能 / s | 固定水电组合 / s | 固定火电组合 / s |
|---|---:|---:|---:|---:|---:|
| Gurobi，2 次均值 | 5.63 | 3.90 | 3.94 | 4.70 | 4.85 |
| HiGHS，2 次均值 | 15.60 | 11.94 | 15.02 | 4.96 | 5.16 |

Gurobi 的水量/储能均有可见影响；HiGHS 主要受联合机组组合整数搜索影响，固定任一大类
组合都会减少搜索复杂度。不能将“固定水电快了”解释为“水电水量方程占了所有时间”。
一次只读用户任务抽样：7 个日窗主求解阶段累计 37.92 s，42 项恢复实验累计 219.74 s；
恢复占阶段累计时间约 85%。此为当时会话快照，不是所有场景的固定倍率。

## Hourly Storage Mode

模型：原 2.6.3.16 中，每时段放电/充电开关为 `d_t,c_t ∈ {0,1}`，
`d_t+c_t≤1`，每小时还满足 `d_t + (Σ_{s∈hour(t)} c_s)/4 ≤ 1`。
因此只要某时段放电，该小时任何时段都不能充电。

当最小充、放电功率均为 0 时，每小时一个 `m_h∈{0,1}` 可精确表达功率投影：
`0≤Pdis_t≤Dmax·a_t·m_h`，`-Cmax·a_t·(1-m_h)≤Pch_t≤0`。
其中 `a_t` 是原始可用状态。任一原可行解可以选取“小时内有放电则 m_h=1，否则 0”；
反向令 `d_t=a_t·m_h,c_t=a_t·(1-m_h)`，即可恢复满足所有原方向/小时约束的整数开关。
停运时 `a_t=0`，不会阻止同小时其余时段运行。96 点之后的两个预测点各自独立。
SOC、效率、循环量、日末目标和其他约束直接保留。闲置时开关代表值可能改变，
但没有开关成本，且定价对零充放电功率的价格扰动界仍为零，因此不改变该零功率点的定价域。
此为功率/SOC/目标投影等价，不宣称整数开关表示唯一相同。

源码：`southern_market.cpp::build_model`。`compact` 模式下、非实时/调频路径、
两个最小功率精确为 0 才准入；正最小功率保持原模式。`reference` 保留原表达供对照。
每个储能整数变量由 `2×98=196` 降到 `24+2=26`；六储能 SCED 从 1176 降到 156，
原时段开关保留为由小时整数变量确定的连续变量，仍执行原模型整数/残差审计。
结果 `compact_storage` 是命中此投影的储能数。

改动前预测：SCED 时间减少至少 25%，完整单日减少至少 15%。初测 Gurobi 单日约 4.96 s，
相对 5.63 s 减少 12%，SCED 约从 1.51 s 降至 1.28 s，减少 15%，未达到预设单项目标。
HiGHS 初测没有明显收益。变量数量不是整数搜索时间的线性预测器；水量/网络 LP 仍存在，
因此此项作为精确模型缩减保留，不能单独宣称已解决整项性能问题。

验证 `[storage_projection]` 对比 compact/reference 的零 GAP 目标、部分时段停运、
正最小功率拒绝投影、独立 SOC 递推和小时方向；另执行既有储能/梯级/实时/调频回归。

## Paired Recovery

固定当日日初 `x_d` 和次日预测 `f_{d+1}` 时，因素 `k` 的恢复结果为
`Y_k = Solve(B, x_d, restore_k(c_d), f_{d+1})`。各 `Y_k` 之间无状态依赖，
可独立计算后按原因素顺序收集；只有主任务结果更新 `x_{d+1}`。
因此并行对象是同一日的独立恢复实验，不是跨日水量或 SOC 递推。

成本模型：总耗时 `T=S+ΣR_k`，两路理想时间 `S+ΣR_k/2`。
按实测恢复占比 85%，理想缩短 42.5%；验收目标为同边界三次中位数至少缩短 25%，
主出清与各实验可行状态一致、原模型残差≤1e-6，目标差异在双方既有 1% GAP 内。
相同目标不要求退化最优解的逐机出力、LMP 或最终库位逐元素一致。

源码：`market_operation.cpp::step_market_operation`。Gurobi 每次构造独立环境，
复核 `../MIPSolvers/src/engine/solver/external/adapters.cpp::GurobiAdapter`。
至多两路；准入边界≤118节点、≤128机组、≤16储能、≤24水库，CPU≥4，且每路线程数≤CPU/2。
用户自动线程 0 在并行恢复内解析为每路 2，显式线程数保持原值；超预算或其他求解器顺序执行。
此界限控制同时存活的 MILP 和 JSON 内存，未将 2000 节点直接投入并行。
日窗内时间限制和 GAP 不变，两个任务各自持有原始完整时限。任一失败保留原失败证据。

输出 `recovery_execution` 记录实验数、workers、请求/实际线程及恢复 wall 秒；
`execution_timing` 区分主出清、恢复 wall、完整日窗 wall 秒，原 `runtime_sec` 语义不变。
GUI 在当日原因详情显示这三个耗时，不把并行阶段累计 CPU/solver 时间当成总等待时间。
`[recovery_parallel]` 将各并行结果与独立单因素顺序重算比较，并检查原输入未被修改和收集顺序。

复现整日工作量：`node tools/market_validation/profile_market_resources.mjs --solvers gurobi
--repetitions 3 --seconds 30 --variants full --recovery --output output/market-performance/recovery-before`。
优化版本以 `--server output/market-performance/overlay/run_gui_server` 指定，使用独立输出目录。

## Zero-Cost Commitment Projection

用户当前保存的随机任务为 1 场景 × 7 日，Gurobi、自动线程、120 s、GAP 0.01、
六项恢复开启。54 台常规机组是 36 水电 + 18 火电，另有 12 风光机组。
只读任务主出清累计 45.071 s，恢复阶段累计 219.740 s；它不是一个 24 s 的单日任务。
该配置在 `output/market-performance/live-input/` 留存，完整任务需另外复现计时。

实现前推导：当水电或新能源机组无开机成本、无启停费用、无最小出力/技术最小出力、无启停曲线、
无最小开停机时间、启停次数限额至少为时点数 T、无预分配调频，且每个时间间隔
的上下爬坡量均覆盖全机最大出力时，令 `u_t=available_t*(1-must_off_t)` 是一个
支配的整数代表。原可行解的出力及分段量原样保留：出力/分段上界和上备用只放宽，
下备用的最小出力项为 0，原上下爬坡行在此代表下冗余；水量、网络与报价行不含新的
开停机限制。由状态差重建 start/stop、stable=u，最多 T 次事件，无新增费用。
反向该代表本身满足原整数模型。因此 SCUC 的功率可行域及最优值不变，开停机代表
可能改变；后续定价固定新的合法代表，不承诺退化最优解的 LMP 或机组组合逐元素相同。
有成本或最低运行限制的机组原样保留。为控制上线范围，火电/抽蓄/其他类型保持原路径，
实时及调频整条调用链不准入。

成本预测：IEEE118 48 台合格机组去除 `48*98=4704` 个自由开停机整数变量，
叠加储能投影后 SCUC 从 6624 降至 1920；连续列和原约束审计保留，额外准入扫描 O(GT)。
整数搜索为非线性成本，验收目标为单日三次中位数相对仅储能优化至少再缩短 25%，
HiGHS 与 Gurobi 分别报告；完整周任务按保存的配置重放。若未达到目标记录为失败，
不调整质量或移除恢复实验。对照投影/未投影 compact 原机组模型的零 GAP 目标、停运状态、
爬坡/最小出力/成本拒绝条件，并要求原模型残差 ≤1e-6。无水库的单机测试中仅将 oracle
类型标为 thermal，使相同电能/备用公式保留自由 u；不改容量、费用和其他参数。
早期完整 reference 大 M 对照在 Release 通过，但 ASan 构建中的 HiGHS 在 reference
求解触发 `HighsDomain.cpp:1705 updateActivityUbChange` 内部断言（新投影先完成且可行）。
这是未解决的旧 reference/debug 路径限制；当前回归 oracle 使用实际生产的未投影 compact
模型，不能将通过结果说成旧 reference 在 sanitizer 下也已通过。

最后独立三轮完整单日（储能投影保持开启，只比较新增开停机投影）：

```bash
node tools/market_validation/profile_market_resources.mjs --server output/market-performance/overlay/run_gui_server --output output/market-performance/commitment-before --repetitions 3 --seconds 15 --variants full
node tools/market_validation/profile_market_resources.mjs --server output/market-performance/commitment-overlay/run_gui_server --output output/market-performance/commitment-after --repetitions 3 --seconds 15 --variants full
```

| 求解器 | 投影前中位数 / s | 投影后中位数 / s | 实际缩短 | 预设至少 25% |
|---|---:|---:|---:|---|
| Gurobi | 4.913 | 4.469 | 9.0% | 未达标 |
| HiGHS | 17.072 | 4.889 | 71.4% | 达标 |

两者均去掉预期 4704 个整数声明；所有目标差 <5.08e-6，残差 <1.52e-10。
Gurobi 偏差触发成本模型复核：先用零 GAP 的原机组模型 oracle 验证投影恒等性（172
断言通过）；再拆开计时，原 SCUC 仅占 1.952/4.913=39.7%，实际 SCUC 减少 20.0%，
所以保持其他阶段时完整链只能减少约 7.9%，而非按整数数量预测至少 25%。HiGHS 的
SCUC 原占 80.9%，减少 87.3%，对应全链约 70.6%，接近实测 71.4%。原因是最初成本
模型把不同求解器的 SCUC 占比和搜索收益混同；数学投影不变。未记录根节点树细节，
不进一步声称具体哪个求解器预处理规则起效。Gurobi 的 25% 单项目标保持失败记录，
不以完整任务的并行收益替代。原始证据及汇总在两目录的 `report.json` / `comparison.json`。

## 完整周任务浏览器验证

命令（每版一次完整周任务；期间没有其他代理启动的求解/编译）：

```bash
node tools/market_validation/replay_market_forecast.mjs --output output/market-performance/live-before
node tools/market_validation/replay_market_forecast.mjs --server output/market-performance/commitment-overlay/run_gui_server --output output/market-performance/live-after
node tools/market_validation/compare_market_replay.mjs output/market-performance/live-before output/market-performance/live-after
```

重放核对导出的实际 base、求解参数和七日随机输入逐项相同，从真实浏览器点击运行计时，
直到七天结果全部显示；包含全部 42 项配对恢复。机器为 Apple M4 Max / 16 logical CPU /
128 GiB，测量时无 swap 或压缩内存。原版本与增量版本的二进制哈希在各 `report.json`。

| 完整工作量 | 原版本 | 三项优化后 |
|---|---:|---:|
| 浏览器点击至完整七日结果 / s | 304.315 | 164.951 |
| 七次主出清累计 / s | 44.163 | 38.879 |
| 主出清 / 恢复实验数量 | 7 / 42 | 7 / 42 |
| SCUC 自由二进制声明数 | 7644 | 1920 |
| 每台储能方向整数声明数 | 196 | 26 |

完整等待减少 45.8%，仍为约 2 分 45 秒，不是“整项任务 20 秒”。新版本七日恢复 wall
合计 124.552 s；累计求解阶段时间不能直接当并行 wall。147 个对应阶段全部可行，
最大原模型残差 3.40e-9；最大相对目标差 0.0095151，低于原有 0.01 GAP。
开停机代表和退化调度可能改变，不宣称结果逐元素相同。浏览器长任务原 109/152 ms、
新 111/161 ms，主要等待来自服务端。此为同机一对完整任务样本，不是通用吞吐保证。

Release 全市场 62 测试通过（29082 断言；随后 oracle 测试单独更新并通过 172 断言）。
投影 ASan/UBSan 1086 断言通过，Gurobi 并行在该 sanitizer 归档不可用而跳过；Release
包含并行对照及自动线程/超预算顺序执行。直接 Node 的 operation、forecast、ancillary、
realtime 及 IEEE118 resources 全部通过。后者独立复算梯级水量、SOC、压力响应，
包含 14 随机日窗及 28 天连续状态承接；交流安全仍失败，不能当作物理安全认证。

## 实际输入参数诊断

复测命令：`node tools/market_validation/replay_market_forecast.mjs --server output/market-performance/commitment-overlay/run_gui_server --output output/market-performance/verification-repeat`。
完整七日第二次为 164.279 s（首次 164.951 s，原版本 304.315 s），降低 46.02%；
`compare_market_replay.mjs` 核对全部 147 阶段通过，最大残差 3.40e-9，最大相对目标差
0.0095151。原版仍只有一次样本，不能解释为跨机器性能保证。

参数诊断 RATIONALE（实现前门槛）：模型保持同一 SCUC/SCED/LMP 稀疏 MILP/LP，
仅对已有 Gurobi 初解、Method、Threads 和认证行预处理选项做对照。
主张：算法参数不改变原约束或质量阈值，但初解的完成成本、障碍法的分解成本和线程开销
可能超过搜索收益。成本账为 `Twall = Tassembly + Tsolve + Taudit + Tother`；
扫描/装配为 O(nnz)，初解审计为 O(nnz)，种子 LP 另加一次求解，整数树及稀疏分解成本
依赖实例，无法由节点数线性预测。机器固定 Apple M4 Max/16 logical CPU，串行运行。
候选验收预测为三次中位数降低至少 10%；未达到即不改默认参数。假设为当前保存的首日、
下一日极值预测点、原 120 s 时限与 0.01 GAP；不移除水库/储能约束。
引用：`southern_market.cpp::solve_scaled/gurobi_method/primal_residual`、本手册上述成本复核，
Gurobi Reference Manual 的 Method、Threads、Start 参数定义。
验证命令：`node tools/market_validation/profile_market_parameters.mjs`，交错次序重复三次，
记录二进制/输入哈希、实际参数、阶段装配/求解/审计及整体 wall；必须三个阶段可行、价格有效、
原单位最大残差 ≤1e-6、对照相对目标差 ≤0.01。只执行首日主链（保留原 week 配置），
关闭恢复实验用于隔离参数，不将此耗时当作完整周任务。adapter runtime 与外部 wall
不是严格互斥的计时区段，差额只能作为待进一步定位的集成开销，不能直接称为 JSON 时间。

三轮参数结果（相同增量二进制，不与编译/其他测试并发）：

| 设置 | 首日主链中位数 / s | 相对 auto | 至少缩短 10% |
|---|---:|---:|---|
| auto | 4.498 | 基准 | 基准 |
| 显式初解 | 6.229 | 慢 38.5% | 未达标 |
| dual simplex | 5.256 | 慢 16.9% | 未达标 |
| barrier | 4.948 | 慢 10.0% | 未达标 |
| 认证行裁剪 | 4.456 | 快 0.9% | 未达标 |
| 2 线程 | 4.456 | 快 0.9% | 未达标 |

全部 54 阶段可行/价格有效，最大原残差 3.63e-8；最大相对目标差 0.006363。
偏差复核：初解确被接受、认证行裁剪确移除 SCUC 10804 行，故不是选项未生效。
初解增加 SCUC 种子 LP（约 0.48 s）且改变整数搜索路径，SCED 也从约 1.34 s 增至
1.94 s；不能因已有前序解就假设热启动必然更快。障碍法使 SCED 约降至 1.14 s，
但 SCUC 约从 1.53 s 增至 2.19 s，抵消收益。行裁剪只减少约 4.1% 非零元，
而求解器内部也做预处理；未测内部各 presolve 规则，不能作更强归因。
该选项按现有契约只裁剪 SCUC/SCED；LMP 保留原行以恢复对偶，脚本分别核对该范围。
改正后的成本判断：当前选项均无 10% 全链收益，默认值保持不变。记录在
`output/market-performance/parameters/report.json`，此筛选不是证明所有模型都适用 auto。

新增计时只观测现有调用，不改约束/参数/求解路径：每阶段常数次 steady_clock 读取，
预期主链计时扰动低于 2%，以同输入 auto 三轮中位数检查。`validation_sec` 为边界校验；
`solve_wall_sec` 包含求解外层缩放/适配器，`solution_export_sec` 为 SCUC/SCED 解映射，
LMP 没有该步骤故为 0。日窗另记 boundary/summary/analysis；恢复记录各自 boundary、
validation、runtime。validation 包含在 main/runtime 内，不能重复相加；并行实验 wall
不能用各实验耗时相加代替。E2E 核对计时非负、阶段之和不超过主链及日窗账目包含关系。

计时版验证命令：

```bash
python3 tools/market_validation/build_market_overlay.py --output output/market-performance/timing-overlay --targets run_gui_server
node tools/market_validation/profile_market_parameters.mjs --server output/market-performance/timing-overlay/run_gui_server --variants auto --output output/market-performance/timing
node tests/e2e/market_operation_e2e.mjs --server output/market-performance/timing-overlay/run_gui_server
node tests/e2e/market_forecast_e2e.mjs --server output/market-performance/timing-overlay/run_gui_server
```

计时版三次主链 wall 为 4.511/4.518/4.498 s，中位数比无计时版 4.498 s 高 0.30%，
低于预设 2% 扰动门槛；这不是新的算法提速。代表样本（中位数对应的第 0 次）分解：

| 互不重叠的计时范围 | 秒 | 说明 |
|---|---:|---|
| 日窗边界生成 | 0.0137 | 当日变更及下一日代表点提取 |
| 边界校验 | 0.0281 | 不包含模型装配 |
| 三阶段模型装配 | 0.6365 | 14.1% 请求 wall |
| 三阶段 solve 外层 | 3.3838 | 75.0% 请求 wall；含 adapter，不另加 runtime |
| 三阶段约束审计/阶段结果 | 0.2630 | 原单位残差、整数与输出 |
| 两次调度解映射 | 0.0629 | SCUC→SCED、SCED→LMP |
| 日摘要及分析 | 0.0157 | 不含恢复实验 |
| 其余 | 0.1072 | 未分段的分配/析构、复制、HTTP 往返等，非纯网络计时 |
| 请求 wall 合计 | 4.5109 | 首日主链，不是完整周任务 |

参数生成+校验仅 0.93%；即使理想化消除也不足 1% 总时间。因此当前实际输入的主要瓶颈
是稀疏模型求解，其次装配，不能把继续等待归因于 JSON 参数处理。SCUC 仍有 152448 列、
202198 行、621566 非零元；1920 自由整数中 1764 来自火电、156 来自储能方向，水电/风光
已无自由开停机整数。水量耦合仍在连续模型中，不能由整数计数断言水电完全不耗时。
后续模型优化应针对装配/连续行列与反复求解，必须保留水量/SOC/网络原约束和误差审计。

计时前后首日所有资源轨迹、96 点结果、lookahead、日初/日末 carry，以及三个阶段
目标、最大残差和模型规模逐项相同。operation GUI/API 回归通过，含恢复计时、七日/月窗、
刷新续跑和移动端；forecast GUI/API 的生成、统计、导出/重载及移动端回归也通过。
计时增量二进制 SHA256 为
`5a19619a4a699ac4b3e144db4f30b8cd859849fefbf4dab03e89356aa2118230`。
现有 8097/8101/8102 服务未替换；新计时字段目前在该独立测试产物中。

## 装配模板与连续系数存储

实现前 RATIONALE：保持原方程生成顺序和逐系数累加顺序。Expr 的小稀疏项由节点分配的
有序树改为有序连续数组；相同列的相加仍按原调用顺序进行。内部变量查询使用
family→稳定设备 ID→时点列号表，字符串名称只用于注册、报告和原路径验证。
每阶段独立、独占借用的有界模板池复用变量/行存储及 CSC 位置；每次仍重新计算所有
系数、右端、界限和目标，不用“某类参数只影响 RHS”的不完整白名单。
只有变量/行布局及每行非零列索引完全匹配时更新 CSC value 数组，否则用原 Triplet
路径重建。投影整数类别变化也使布局失效。每个 SCUC/SCED/LMP 最多保留两个模板，
仅缓存 ≤118 母线/128 机组/16 储能/24 水库的独立日前链；实时、辅助服务和更大系统
不复用模板。参考模式保留原 map 表达式/字符串查找/Triplet 装配。
为限制常驻内存，每个模板还要求列数 ≤250000、行数 ≤400000、非零项 ≤2000000，
否则释放。阈值覆盖本算例 152448/202198/621566 并留约两倍行容量；不是数学准入条件。

数学依据：在同一有序行列空间中，稀疏矩阵是结构 (outer, inner) 与数值 values 的组合；
只有逐项结构相同才可复用前者。更新全部 values/RHS/c/lb/ub 后与重新装配完全相同。
连续有序项与原有序 map 的遍历序相同，重复项累加顺序不变，因此以精确 double 相等
而非放宽容差核对。参照 Eigen SparseMatrix compressed storage / setFromTriplets，
以及本文件与 `Build::finish/Expr::add` 的原实现。

成本模型：原有热点包含 O(K log K) 字符串树查询及每系数分配，Triplet→CSC 每阶段
O(nnz) 另需分配/排序；新路径使用短有序数组和 O(nnz) 原位结构核对及数值写入。
单条长行的有序数组插入最坏 O(k²)，这是需用实际非零分布测量的边界，不声称普遍线性。
原首日装配 0.6365 s，预设目标 ≤0.32 s（三次中位数），若其他阶段不变首日 wall
预计至少减少约 7%；完整七日三次装配链与 42 恢复仍全部执行，不预先保证相同百分比。
验证模式逐元素比较两路 A/Aeq/c/b/beq、变量界限/类型/名称、整数及审计集合、原单位
行式/缩放、费用与指标映射，失败直接抛错。对照真实七日原路径/缓存路径目标、原残差、
资源轨迹与水量/SOC 承接；初解、求解参数及矩阵顺序保持一致。发布性能不用验证模式的
重复装配计时。新增 `[assembly_cache]` 回归覆盖数值变化、结构失效和缓存交错；
命令为 `test_southern_market '[assembly_cache]'`、ASan/UBSan 同标签及已有 operation/
forecast/resources 回归，真实任务以 `replay_market_forecast.mjs` 重放。

第一版实测（`assembly-profile/`）：三次原路径装配中位数 0.6535 s、缓存 0.4779 s，
减少 26.9%，未达到 ≤0.32 s；首日 wall 中位数 4.5116→4.3054 s，减少 4.57%，
未达最初约 7% 预测。9 个 verify 阶段与原路逐元素精确相同，所有目标/残差相同，
第二、三次缓存阶段均复用两矩阵，排除模板未生效。成本模型最初高估了树查询与
Triplet 构造的占比：仍需重新计算方程、生成行名、执行边界冗余证书及规模报告，
CSC 更新也必须扫描每个非零项。新账目用实际可节约 0.176 s，预测 wall 约 3.9%，
加上约束遍历/析构差异与观测 4.57% 接近；不把剩余部分称为已被模板消除。
原来的减半门槛保留失败记录；后续整周测量独立报告，不以关掉恢复/LMP 达标。

最终完整工作量对照（同一 assembly-final 二进制，两个性能任务串行运行，期间无本代理
启动的其他编译/求解）：

```bash
node tools/market_validation/replay_market_forecast.mjs --server output/market-performance/assembly-final/run_gui_server --assembly reference --output output/market-performance/assembly-week-reference
node tools/market_validation/replay_market_forecast.mjs --server output/market-performance/assembly-final/run_gui_server --assembly cached --output output/market-performance/assembly-week-cached
node tools/market_validation/replay_market_forecast.mjs --server output/market-performance/assembly-final/run_gui_server --assembly verify --output output/market-performance/assembly-week-verify
node tools/market_validation/compare_market_assembly.mjs output/market-performance/assembly-week-reference output/market-performance/assembly-week-cached output/market-performance/assembly-week-verify
```

| 七日、7 主链＋42 恢复 | 原装配 | 缓存装配 | 缩短 |
|---|---:|---:|---:|
| 浏览器完整等待 / s | 165.664 | 158.446 | 4.36% |
| 主出清累计 / s | 39.090 | 37.502 | 4.06% |
| 恢复实验 wall 累计 / s | 125.043 | 119.535 | 4.41% |
| 147 阶段装配累计 / s | 32.911 | 24.143 | 26.64% |

装配累计含并行 worker，不能直接从完整 wall 减去 8.767 s。缓存复用了 294 个候选
矩阵中的 282 个；第一次为每阶段的两份独占模板创建矩阵，共 12 个未复用。
原路与缓存的 147 个阶段目标/最大残差及七日资源轨迹、节点价格、线路结果、预测点、
日初/日末 carry 逐项完全相同。这里是同一二进制的装配对照；与上一轮旧二进制比，
资源与状态仍完全相同，但有至多 1.87e-9 的目标和 3.40e-11 的残差报告舍入差。
不把它说成跨编译产物逐 bit 相同。每模式只有一次完整周性能样本。
逐元素 verify 是额外正确性任务，可与资源回归并行，其耗时不用于提速结论。
完整 verify 的 147 阶段/294 矩阵全部精确匹配；七日资源、价格、线路、预测与 carry
仍与原路径相同。额外验证运行的标量报告有最大 1.8627e-9 目标差和 3.3981e-11 残差差，
因此“verify 运行所有标量逐 bit 相同”检查失败，报告明确保留
`verify_scalar_bitwise_equal=false`；两项差值均低于 1e-6，矩阵检查仍是精确相等。
源码目标由 Eigen `c.dot(x)` 汇总，残差由原式复算；未导出所有内部列向量，不能进一步
把差异唯一归因于求解器或点积累加。原路径/缓存两个性能运行的对应标量则完全相同。

最终回归：Release 全市场 64 测试/29176 断言，ASan/UBSan 装配缓存 2 测试/88 断言，
operation/forecast GUI/API 全部通过。IEEE118 resources 独立复算水量/SOC，验证压力、
停运、14 个随机日窗及 28 天连续承接，通过；交流安全仍返回 ac_security_failed。
构建源与记录哈希一致，未改求解器依赖或构建准入保护。缓存以额外常驻内存换装配成本，
资源回归期间采样 RSS 约 1.43 GiB、双路径 verify 约 2.31 GiB，含求解器及任务 JSON，
不是缓存独占内存或峰值估计；采样在 `assembly-week-verify/process-sample.json`。

## 原因分析触发与恢复定价隔离

RATIONALE（实施前门槛）：模型为相同日初状态下单因素潜在结果差
`Delta_f = metric(SCED(x)) - metric(SCED(restore_f(x)))`；主链仍含 LMP。
恢复供需指标仅依赖 SCED，故省略其后的定价不改变这些指标或承接状态。
异常触发取已实现 96 点中缺额、富余、线路越限和任一值超过 1e-6 MW，
与现有界面诊断阈值一致；不包含 D+1 预测点，也不证明未执行的反事实差为零。
成本为主链加被请求的恢复链；配对 wall 为每对耗时最大值之和。
基线 158.446 s 中主链 37.502 s、恢复 119.535 s；同一 IEEE118 保存任务无异常，
新自动模式预期整周 38--45 s，接受 <=50 s；始终恢复但不定价预测 139 s，
接受 <=145 s。假设同机、同参数、无竞争负载，均保留原 SCUC/SCED 数学模型。
参考本文件 Paired Recovery、南方规则 2.6 SCUC/SCED 与独立定价流程，
源码 `step_market_operation` / `run_southern_market_impl`。
验证：`test_southern_market '[recovery_policy]'`、operation/forecast E2E、
`replay_market_forecast.mjs` 完整七日；核对主链目标/残差 <=1e-6、资源/价格/承接
逐项一致，恢复 dispatch_only/full 的供需指标一致，手动补算不推进天数或修改 carry。

契约映射：config.explain 保留开关；explain_trigger 为 anomaly/always/manual，
新 GUI 和预测默认 anomaly，缺省字段的旧配置按 always 兼容。recovery_pricing 为
dispatch_only/full，缺省 dispatch_only。每日日志 cause_analysis.status 区分
not_requested/not_triggered/unavailable/completed/partial_failure/no_interventions；
未执行不生成零差值。手动 action=explain 指定历史 day（预测还含 scenario）及 pricing，
使用保存 state_start 和原下一日预测，保留原运行状态、主结果和后续承接。
恢复证据声明 pricing_scope/prices_valid，dispatch_only 不含 lmp 阶段。
solver_timing 秒值来自 Gurobi adapter 的线程局部最近调用：模型导入、GRBoptimize、
提取结果；市场侧单独测环境初始化。未执行阶段和其他后端为 null；
optimize 包含预处理、根 LP、搜索，不伪称独立整数搜索时间。
新 getter 不改变 Adapter/Options/SolveResult 类布局，避免破坏现有归档 ABI。

## 构建与范围

验收中发现的价格非一致性（保留失败证据）：首次 dispatch-only 全周对照的第 4/5 日
主 LMP 与 full 不逐 bit 一致，第 5 日最大价格差 6.64876 CNY/MWh，不是舍入误差。
资源、线路、日初/日末水量/SOC、供需指标完全相同；LMP 目标最大差 4.47e-8、
残差报告差 1.12e-10。auto/verify 与 full 的主价格及所有资源则完全相同。
原“所有模式价格逐项一致”门槛在该样本失败，不能通过放宽价格容差隐藏。
需先排查模型不一致，再排查 Gurobi Method=-1 的并发 LP 算法及非唯一对偶选择。
诊断协议：固定原第 5 日初状态及 D+1 预测，重复相同完整主链，不运行恢复；
记录 Gurobi model fingerprint、算法日志和价格，检查相同模型/调度是否可产生不同对偶。
此诊断不改默认求解算法或价格定义，也不作为性能样本。
同输入自动法八次均复现 full 价格；日志显示并发 LP 中 barrier 胜出，LMP fingerprint
均为 0x0819db94。下一诊断仅装配一份固定 SCED 的 LMP，对同一对象分别用已有 auto /
dual_simplex / barrier 求解，避免根 LP 策略改变 SCUC/SCED。线性规划强对偶只保证最优
目标一致，不保证最优对偶唯一；预测各目标差 <=1e-6，而 dual-simplex 可能复现 6.65
价差。依据 Gurobi Method=-1 concurrent LP 语义和本契约的条件 LMP 定义；不改变生产
默认参数，诊断源码 `tools/market_validation/probe_market_lmp_duals.cpp`。

诊断闭环：同一份 Build 的同一 LP，Gurobi fingerprint 均为 `0x0819db94`。
auto（并发中 barrier 胜出）精确复现 full；dual_simplex 精确复现 dispatch-only 那次
第 5 日的全部节点价格，二者最大价差 6.648759989621006 CNY/MWh。目标分别为
14034457.139619324 / 14034457.13961928，残差 4.77e-12 / 1.16e-10。
单独 barrier、Crossover=0 另给出一组条件价格，目标 14034457.139619466、残差
1.15e-9。证据在 `recovery-lmp-repeat/{dual-comparison.json,dual-methods.log}`；
一份固定矩阵即可复现该差异，排除恢复跳过 LMP 导致的 SCED 承接错误。
原因是最优对偶非唯一及自动并发算法选择，原推导漏了“最优对偶唯一或选取策略确定”
这一前提。数学等价不推出价格逐 bit 一致，不能把 6.65 归为舍入；原价格一致门槛保留
`strict_price_identity=false`，比较脚本因此返回 1。生产求解算法/参数未改动，
未实现独立的规范化价格选择规则。默认模式本次主价格恰与 full 精确相同，不承诺未来
每次自动 LP 都取相同最优对偶。教程及结果区明确说明条件价格的这一边界。

最终同版实测（性能模式逐个串行，无本代理并行编译/求解；verify 是额外正确性检查）：

| 七日模式 | 主链 / 恢复 | 恢复 LMP 次数 | 浏览器等待 / s |
|---|---:|---:|---:|
| always + full | 7 / 42 | 42 | 157.075 |
| always + dispatch_only | 7 / 42 | 0 | 136.086 |
| anomaly + dispatch_only（新默认） | 7 / 0 | 0 | 38.562 |

全部模式主 LMP 仍执行 7 次。默认减少 75.45%（预测 38--45 s，门槛 <=50 s 达到）；
仅省略恢复定价减少 13.36%、20.989 s（预测约139 s，门槛<=145 s达到）。该零异常
算例的跳过比例不能推广到异常周；所有恢复仍执行时仍需约2分16秒。
主资源、水量/SOC 承接、线路和供需指标逐项完全相同，42 个恢复的全部供需差值相同。
147 个被对照阶段最大目标差 4.47e-8、残差差 1.12e-10；主价格差及非唯一性见上文，
不宣称全部数值逐 bit 一致。verify 的 21 主阶段 / 42 矩阵与原装配精确相同。
第 3 日手动补算 6 个因素耗时17.986 s，七日主结果、统计、原耗时和前后承接完全不变。
内部主链合计：环境0.0132 s、导入1.0813 s、GRBoptimize 30.2689 s、提取/释放0.0088 s；
优化调用约占 solve_wall 的96.5%，不能再把其中全部称为模型导入或整数搜索。

```bash
node tools/market_validation/replay_market_forecast.mjs --server output/market-performance/recovery-final/run_gui_server --assembly cached --trigger always --pricing full --output output/market-performance/recovery-week-full
node tools/market_validation/replay_market_forecast.mjs --server output/market-performance/recovery-final/run_gui_server --assembly cached --trigger always --pricing dispatch_only --output output/market-performance/recovery-week-dispatch
node tools/market_validation/replay_market_forecast.mjs --server output/market-performance/recovery-final/run_gui_server --assembly cached --trigger anomaly --pricing dispatch_only --manual-day 2 --output output/market-performance/recovery-week-auto-final
node tools/market_validation/replay_market_forecast.mjs --server output/market-performance/recovery-final/run_gui_server --assembly verify --trigger anomaly --pricing dispatch_only --output output/market-performance/recovery-week-verify
node tools/market_validation/compare_market_recovery.mjs output/market-performance/recovery-week-full output/market-performance/recovery-week-dispatch output/market-performance/recovery-week-auto-final output/market-performance/recovery-week-verify
node tools/market_validation/probe_market_lmp_repeat.mjs
MIPSOLVERS_GUROBI_VERBOSE=1 output/market-performance/recovery-lmp-repeat/probe output/market-performance/recovery-lmp-repeat/input.json output/market-performance/recovery-lmp-repeat/dual-methods.json
```

最后一项诊断的编译/链接命令保存在 `recovery-lmp-repeat/build.json`。
新服务 `http://127.0.0.1:8104/xjtu/#market-operation` 使用上述最终二进制，PID58652；
保存原IEEE118边界/seed，并将策略显式设置 anomaly/dispatch_only，启动 ready 0/7。
桌面/390px移动端策略控件、结果区和内部计时表已实览，未发现新增页面横向溢出；
启动记录、日志、截图在 `recovery-service/`，原8097/8101/8102/8103服务保留。

恢复策略版本使用 `output/market-performance/recovery-final/`，重编译四个市场单元
（含 market_forecast）、HTTP server 及 MIPSolvers external/adapters.cpp；其余归档对象
与第三方库沿用既有构建。`build.json` 记录源码/头/两份主归档/二进制哈希和全部命令。
这是本地增量实验，未修改依赖准入保护，也不是干净发布构建。机器为 Apple M4 Max、
128 GiB，macOS arm64 Release -O3；HySim HEAD 为
`8b93145bf4f5617bdf5d8eb7856a93570920fb01`，MIPSolvers HEAD 为
`e6c932e5f8a409cc87bf2668b86eb895f3eccea5`，两者均含已记录的未提交修改。
最终 server SHA256 为 `59c415d2c21c28b3a13ecea3a8f066b1005b60a0d5e03699077ea7063337ad5e`。

该版本 Release 全市场 66 测试 / 29262 断言通过；ASan/UBSan recovery_policy
2 测试 / 53 断言通过，detect_leaks=0，sanitizer 构建无 Gurobi，因此不覆盖其内部计时。
Release 专项覆盖实际 Gurobi 三阶段非负计时及上层 wall 边界。
operation/forecast GUI/API 回归通过，包含无异常未触发、异常归因、历史日两种范围
补算、原状态/统计不变、无效索引/旧 run_id 拒绝、导出/重载、桌面/移动端和旧接口兼容。
旧配对回归最初因强制读取被省略的 lmp 而失败，已将该完整链对照显式设置 full，
供需归因的缺席 LMP 由独立专项断言，不删除原目标/残差检查。
编译警告来自已有 DynamicSystem class/struct、服务器未用变量/参数和外部 SCIP/Ipopt，
本轮未处理这些无关警告。未运行 MIPSolvers 独立全库测试。

本地性能构建由 `tools/market_validation/build_market_overlay.py` 编译变更的市场翻译单元并链接
现有归档和后端对象；新测试源同时重编译。它不修改现有服务、归档、CMake 或依赖保护，
产物是明确标记的增量本地实验，不是从干净依赖重建的发布版。
早期两单元构建在本轮增加 southern_boundary，共三单元，以纳入 assembly_mode schema。
`build.json` 记录全部命令、源码和归档/二进制哈希；未变的 C++ 对象沿用现有基线。
完整回归、实际性能、sanitizer 和启用服务见[开发状态](../../overview/development_status.md)。

最新装配优化服务：`http://127.0.0.1:8103/xjtu/#market-operation`，运行
`output/market-performance/assembly-final/run_gui_server`，SHA256
`92b13f4329e79ee3a690c1f6ff3bc9194a3eecb58fec54137b8d9bd675a0ec6c`。
同一保存边界显式选择 cached，并生成同 seed 完整周任务；启动时 ready、0/7，未自动运行。
日志和启动记录位于 `output/market-performance/assembly-service/`。

此前优化服务：`http://127.0.0.1:8102/xjtu/#market-operation`，运行
`output/market-performance/commitment-overlay/run_gui_server`。新会话载入保存的原始边界和
同 seed 的完整周配置，启动时为 ready。原 8097/8101/8102 会话及结果均保留；
旧进程不会因源码更新而自动获得新算法。启动记录与日志位于 `output/market-performance/service/`。
