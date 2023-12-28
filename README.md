#运行a——1即可
为了充分发挥多机编队的优点，加强编队飞行的灵活性以应对复杂、突发的 应用场景，多机编队协同避障问题与多机自主编队跟踪控制问题成为亟待解决的 关键问题。本文以实现多机精确可靠编队飞行为研究重点，结合实际无人机编队 飞行任务需求，设计无人机精确编队协同避障与跟踪控制策略，综合解决编队个 体之间以及与周围环境障碍物的防撞安全性，在此基础上构建数值仿真系统，对 所研究方法进行仿真验证，充分挖掘复杂环境下多机编队飞行能力。
 针对多飞行器快速响应任务需求，研究基于优化算法的多飞行器队形构成方 法，形成可行的编队飞行方案。
 针对巡航阶段静态多障碍物环境及动态障碍物环境下实时避障的需求，开展 多机动态协同避障方法研究。
 在多机编队飞行场景下，研究多机自主编队保持技术，完成多机自主协同编 队飞行任务，并开展多机编队保持控制仿真验证。
总体实施方案如下：
（1）针对多无人机编队快速任务响应队形变换需求，采用智能优化算法对 多机编队队形优化与重构进行研究，快速规划无人机编队队形重构过程中的运动 轨迹，满足实时性需求和不同任务的多机编队需求，以保证编队整体规划路径的 可飞行性。
（2）针对巡航阶段多机编队队形保持的需求，采用虚拟结构法来实现多机 自主分布式编队保持控制，在虚拟长机状态估计算法的基础上，设计了具有线性 混合器的分布式编队保持控制器。（3）针对协同避障问题，设计基于改进人工势场的无人机编队避障算法， 将人工势场应用到空间范围。建立人工势场后基于动力学条件进行编队飞行，目 标点对无人机施以引力，障碍物对无人机产生斥力，使无人机在障碍物环境中顺 利到达目的地，可以满足不同障碍物场景的多机编队飞行需求。同时，针对动态 障碍物环境，设计一种改进的动态窗口法以保证在遭遇突发威胁时多机编队能够 根据实时环境信息在有限的时间及计算资源的条件下，规划出躲避威胁的可行航 迹。
（4）本课题的仿真系统在 Matlab/Simulink 中进行搭建，首先根据已知的气 动力模型、几何数据模型、质量特性模型、控制律模型及作动器模型等构建飞行 器的单机控制仿真模型。然后基于该模型，通过编队控制器、避障控制器及轨迹 跟踪控制器等搭建编队控制与避障系统，在数值仿真上分析方案的可行性与控制 精度，对关键技术做出验证和评估结论。