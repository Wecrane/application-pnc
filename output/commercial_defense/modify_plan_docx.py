# -*- coding: utf-8 -*-
"""
计划书.docx -> 计划书_Apollo版.docx
把百度 Apollo 开源平台融入矿山机器人运输项目，突出 Apollo 作用：
- 段落级文本替换（保留段落格式）
- 插入新专节（3.2.4 技术底座 / 3.6.4 平台化生态壁垒 / 6.7 生态协同 等）
- XML 层替换文本框内团队成员介绍
用法: python3 modify_plan_docx.py
"""
import docx
from docx.oxml.ns import qn
import copy, zipfile, shutil, os, sys

SRC = '/home/skye/文档/计划书_Apollo版.docx'  # 已是备份副本，直接原地改

# ---------------- 段落级文本替换 ----------------
REPLACEMENTS = [
    # 1 项目概述
    ("智采飞扬-矿山机器人智能开采运输装备研发与应用",
     "智采飞扬——基于百度Apollo开源平台的矿山机器人智能开采运输装备研发与应用"),
    ("项目已获得 5 项专利技术、9 篇专业论文、2 项软件著作、3项省部级奖项；团队成员累计发布论文 40 余篇，担任省部级就创业课题 10 余项。",
     "项目已获得 5 项专利技术、9 篇专业论文、2 项软件著作、3项省部级奖项；团队成员累计发布论文 40 余篇，担任省部级就创业课题 10 余项。项目团队基于百度 Apollo 开源平台开展智能驾驶场景化开发，掌握 Apollo Planning、Control、CyberRT 全栈开发与调优能力。"),
    ("项目实施1年来，在露天矿机器人化装运卸标准",
     "项目实施1年来，基于百度 Apollo 开源平台，在露天矿机器人化装运卸标准"),
    ("面向智慧矿山开采运输设备研发战略需求和国际学术前沿，以智能矿山平行管理与智能控制、基于机器人技术的先进控制、自动驾驶为应用研究方向",
     "面向智慧矿山开采运输设备研发战略需求和国际学术前沿，以智能矿山平行管理与智能控制、基于机器人技术的先进控制、百度 Apollo 开源平台与自动驾驶为应用研究方向"),
    # 3 技术概述
    ("车载软件系统包括感知与定位、决策与规划、控制、人机交互、安全监控和数据存储等模块。",
     "车载软件系统基于百度 Apollo 开源平台的感知（Perception）、定位（Localization）、规划（Planning）与控制（Control）模块构建，并结合矿山场景进行二次开发，涵盖感知与定位、决策与规划、控制、人机交互、安全监控和数据存储等功能。"),
    ("装载区路径规划是指为无人矿卡从入场点到装载点、再到出场点设计连续且符合车辆运动学约束的全局路线。",
     "装载区路径规划基于 Apollo Planning 框架，是指为无人矿卡从入场点到装载点、再到出场点设计连续且符合车辆运动学约束的全局路线。"),
    ("本研究在混合 A* 路径规划方法基础上提出回旋曲线组合搜索方法。",
     "本研究在百度 Apollo 开源平台基础上，针对矿山场景在混合 A* 路径规划方法基础上提出回旋曲线组合搜索方法。"),
    ("端边感知系统由车端传感系统和路侧传感系统共同构成。",
     "端边感知系统基于 Apollo Perception 多传感器融合框架构建，由车端传感系统和路侧传感系统共同构成。"),
    ("MPC 控制器根据车辆动力学约束、路径曲率和边界安全距离输出控制量，实现卸载区精准停靠。",
     "基于 Apollo Control 模块的 MPC 控制器根据车辆动力学约束、路径曲率和边界安全距离输出控制量，实现卸载区精准停靠。"),
    ("通信系统基于 5G 与 V2X，实现车-云、车-车、车-路通信，承担调度指令",
     "通信系统基于 5G、V2X 与百度 Apollo CyberRT 通信框架，实现车-云、车-车、车-路通信，承担调度指令"),
    # 4 产品与服务
    ("产品采用自主导航、智能调度与高精度定位等前沿技术",
     "产品基于百度 Apollo 开源平台，采用自主导航、智能调度与高精度定位等前沿技术"),
    ("1）自主导航模块：采用混合 A*算法与回旋曲线组合搜索技术",
     "1）自主导航模块：基于 Apollo Planning 平台，采用混合 A*算法与回旋曲线组合搜索技术"),
    ("2）精准定位与自动对接：依托多传感器（GPS、激光雷达、视觉传感器等） 数据融合技术",
     "2）精准定位与自动对接：依托百度 Apollo Localization 平台的多传感器（GPS、激光雷达、视觉传感器等）数据融合技术"),
    ("3）多机智能协同：通过建立车辆与机械臂的协同作业场景库和冲突预测模型",
     "3）多机智能协同：基于 Apollo CyberRT 通信框架与调度能力，通过建立车辆与机械臂的协同作业场景库和冲突预测模型"),
    ("4）数据采集与决策平台：系统内嵌数字孪生技术",
     "4）数据采集与决策平台：系统内嵌基于 Apollo 云仿真的数字孪生技术"),
    # 5 市场分析
    ("目前这项技术仅为公司独有，且该技术在世界领先，有较强的竞争能力。",
     "本系统的矿山场景算法为团队基于百度 Apollo 开源平台自主研发，结合 Apollo 平台生态与产业协同能力，具有较强的差异化竞争能力。"),
    # 6 商业模式
    ("依托中国矿业大学（北京）高校平台，自主研发高精度定位算法，解决矿区信号遮挡问题",
     "依托中国矿业大学（北京）高校平台与百度 Apollo 开源生态，基于 Apollo 平台开发高精度定位算法，解决矿区信号遮挡问题"),
    # 12 团队（正文部分）
    ("截至目前，我们的项目已成功获得5项专利技术授权，发表了9篇高质量的专业论文，并取得了2项软件著作权。",
     "截至目前，我们的项目已成功获得5项专利技术授权，发表了9篇高质量的专业论文，并取得了2项软件著作权。团队已基于百度 Apollo 平台完成全国决赛仿真与实车任务，具备平台级工程开发与调优实战经验。"),
    # 13 未来战略
    ("数字孪生平台完善：积累 100 个以上矿区地形数据，构建全球最大矿山仿真数据库",
     "数字孪生平台完善：基于 Apollo 云仿真平台积累 100 个以上矿区地形数据，构建全球最大矿山仿真数据库"),
]

# ---------------- 插入点 ----------------
# (锚点子串, 插入位置 before/after, [(kind, text), ...])  kind: heading/body
INSERTIONS = [
    ("我国在矿山无人驾驶技术方面起步较晚，但近年来发展迅速，部分关键技术已取得突破。", "after", [
        ("body", "在开源平台层面，百度 Apollo 开源平台是全球领先的自动驾驶开源平台，提供感知、定位、规划、控制、通信（CyberRT）与云仿真全栈能力，已支持园区、矿区等封闭场景落地验证。基于 Apollo 开源平台进行场景化二次开发，已成为国内智能驾驶行业降低研发门槛、加速产业化落地的重要路径。本项目即以百度 Apollo 开源平台为核心技术底座，聚焦露天矿装运卸场景开展二次开发与应用。"),
    ]),
    ("从国际应用情况看，无人驾驶运输技术能够提升矿山运输效率、降低人工操作风险，并为矿山生产组织方式的重构提供技术基础。", "after", [
        ("body", "在国内，百度 Apollo 开源平台已形成完整的自动驾驶技术栈，并在园区、矿区等封闭场景得到验证，为矿山无人驾驶提供了成熟、可靠、可扩展的技术底座。本项目基于 Apollo 平台开展矿山场景二次开发，显著缩短基础自动驾驶能力的研发周期，将研发资源集中于露天矿特有的车-铲协同、挡墙检测与群智调度等场景算法。"),
    ]),
    ("支撑多车在装载区和卸载区协同运行，并满足实际矿山常态化生产的安全性、稳定性和可维护性要求。", "after", [
        ("body", "在技术边界上，本项目通用自动驾驶能力（感知、定位、规划、控制、通信、仿真）基于百度 Apollo 开源平台构建；露天矿车-铲精准对位、卸载区挡墙自适应检测、多车协同与群智调度等矿山特性算法由团队基于 Apollo 平台自主研发，形成差异化技术壁垒。"),
    ]),
    ("3.3车-铲精准对位与装载区路径规划技术", "before", [
        ("heading", "3.2.4基于百度Apollo开源平台的技术底座"),
        ("body", "本项目采用“通用平台+场景自研”的研发路线，以百度 Apollo 开源平台为自动驾驶核心技术底座，聚焦露天矿装运卸场景开展二次开发。Apollo 平台提供感知（Perception）、定位（Localization）、规划（Planning）、控制（Control）、通信（CyberRT）与云仿真（DreamView/云仿真）全栈能力，并支持场景（Scenario）、任务（Task）插件化扩展机制，为本项目矿山场景算法开发提供了坚实基础。"),
        ("body", "Apollo Perception：多传感器融合感知，扩展点云网格构建与挡墙边缘语义分割；"),
        ("body", "Apollo Localization：高精度定位（RTK+点云融合、SLAM），支撑车-铲坐标统一；"),
        ("body", "Apollo Planning：场景（Scenario）/任务（Task）扩展机制，实现混合 A*、回旋曲线组合搜索的装/卸区路径规划；"),
        ("body", "Apollo Control：MPC 轨迹跟踪与装卸区精准停靠控制；"),
        ("body", "Apollo CyberRT：车-云、车-车、车-路通信与多机协同数据交互；"),
        ("body", "Apollo 云仿真/DreamView：数字孪生、平行仿真与装运卸全流程虚拟验证。"),
        ("body", "平台与自研的边界：通用自动驾驶能力基于 Apollo 平台构建；露天矿车-铲精准对位、卸载区挡墙自适应检测、多车协同调度、群智调度、健康管理与远程应急接管等矿山特性算法为团队基于 Apollo 平台自主研发，形成差异化技术壁垒。"),
    ]),
    ("4 产品与服务", "before", [
        ("heading", "3.6.4平台化生态壁垒"),
        ("body", "本项目以百度 Apollo 开源平台为技术底座，可紧跟智能驾驶产业链技术演进，依托 Apollo 开源生态获取持续的技术迭代与人才供给，并与百度生态内合作伙伴协同拓展矿山场景应用，形成“平台+场景”的复合竞争壁垒。"),
    ]),
    ("智能决策平台：借助云端大数据分析和数字孪生技术，对装卸作业进行实时监控与智能调度，帮助企业不断优化作业流程、降低成本，并实现作业效益的持续提升。", "after", [
        ("body", "开源生态与产业协同优势：基于百度 Apollo 开源平台，产品紧跟智能驾驶产业链技术演进，可便捷获取 Apollo 生态的算法、工具链与产业资源，实现持续迭代与生态协同，并具备向百度 Apollo 生态伙伴体系发展的潜力。"),
    ]),
    ("7 应用成果", "before", [
        ("heading", "6.7与百度Apollo生态的协同"),
        ("body", "本项目深度融入百度 Apollo 生态：技术底座基于 Apollo 开源平台；算法迭代依托 Apollo 开源社区；产品可与 Apollo 生态内合作伙伴协同集成，共同服务矿区无人化需求；项目成果可反哺 Apollo 矿区场景应用。项目符合百度 Apollo 生态伙伴体系的发展方向，具备较强的生态适配与发展潜力。"),
    ]),
    ("示范应用表明，机器人化采运系统在运行效率提升、作业安全保障、特殊场景适配和生产工艺优化方面形成了体系化成果。", "after", [
        ("body", "上述感知、定位、规划、控制链路均基于百度 Apollo 开源平台构建，并在 Apollo 仿真平台完成与实车数据一致的场景复现验证。"),
    ]),
]

# ---------------- XML 层替换（文本框内团队成员） ----------------
XML_REPLACEMENTS = [
    ("擅长多传感器融合定位与环境感知技术，精通混合A*与回旋曲线组合搜索的路径规划算法",
     "熟练掌握百度 Apollo 开源平台（CyberRT/Planning/Control）开发与调优，擅长多传感器融合定位与环境感知技术，精通混合A*与回旋曲线组合搜索的路径规划算法"),
    ("擅长ROS/ROS2系统开发与机器人底层控制逻辑，精通基于ICP与PL-ICP算法的SLAM建图技术",
     "熟悉百度 Apollo 开源平台开发框架，擅长ROS/ROS2系统开发与机器人底层控制逻辑，精通基于ICP与PL-ICP算法的SLAM建图技术"),
]

def replace_par_text(p, old, new):
    """段内合并文本替换，写回第一个 run。返回是否成功。"""
    full = ''.join(r.text for r in p.runs)
    if old not in full:
        return False
    newfull = full.replace(old, new)
    if p.runs:
        p.runs[0].text = newfull
        for r in p.runs[1:]:
            r.text = ''
    return True

def make_para(template_p, text):
    """基于模板段落 deepcopy 一个新段落（保留段落+首个run格式）。"""
    np = copy.deepcopy(template_p._p)
    for r in np.findall(qn('w:r')):
        np.remove(r)
    src_runs = template_p._p.findall(qn('w:r'))
    if src_runs:
        run = copy.deepcopy(src_runs[0])
        for t in run.findall(qn('w:t')):
            run.remove(t)
        t = run.makeelement(qn('w:t'), {})
        t.text = text
        run.append(t)
        np.append(run)
    else:
        r = np.makeelement(qn('w:r'), {})
        t = np.makeelement(qn('w:t'), {})
        t.text = text
        r.append(t)
        np.append(r)
    return np

def main():
    doc = docx.Document(SRC)
    paras = doc.paragraphs

    # ---- 替换 ----
    ok, fail = [], []
    for old, new in REPLACEMENTS:
        done = 0
        for p in paras:
            if replace_par_text(p, old, new):
                done += 1
        if done == 1:
            ok.append(old[:30])
        elif done == 0:
            fail.append(('NOT_FOUND', old[:40]))
        else:
            fail.append(('MULTI(%d)' % done, old[:40]))

    # ---- 插入 ----
    for anchor, where, items in INSERTIONS:
        hit = [p for p in paras if anchor in p.text]
        if len(hit) != 1:
            fail.append(('ANCHOR(%d)' % len(hit), anchor[:40]))
            continue
        anchor_p = hit[0]
        # 模板：标题用含"3.2.3六大关键系统架构"的段落，正文用锚点段落自身
        heading_tpl = next((p for p in paras if '3.2.3六大关键系统架构' in p.text), anchor_p)
        body_tpl = anchor_p
        prev = anchor_p
        for kind, text in items:
            tpl = heading_tpl if kind == 'heading' else body_tpl
            new_p = make_para(tpl, text)
            if where == 'after':
                prev._p.addnext(new_p)
            else:  # before: 依次插到 anchor 前面，保持顺序
                anchor_p._p.addprevious(new_p)
            prev = new_p
        ok.append('INSERT@' + anchor[:24])

    doc.save(SRC)
    print('== 段落级替换 ==')
    for o in ok:
        print('  OK  ', o)
    for st, m in fail:
        print('  FAIL', st, m)

    # ---- XML 层替换（文本框） ----
    tmp = SRC + '.tmp'
    with zipfile.ZipFile(SRC) as zin, zipfile.ZipFile(tmp, 'w', zipfile.ZIP_DEFLATED) as zout:
        for item in zin.infolist():
            data = zin.read(item.filename)
            if item.filename == 'word/document.xml':
                xml = data.decode('utf-8')
                for old, new in XML_REPLACEMENTS:
                    n = xml.count(old)
                    if n:
                        xml = xml.replace(old, new)
                        print('  XML 替换 %d 处: %s...' % (n, old[:30]))
                    else:
                        print('  XML FAIL 未找到: %s...' % old[:30])
                data = xml.encode('utf-8')
            zout.writestr(item, data)
    shutil.move(tmp, SRC)
    print('== 完成 ==', SRC)

if __name__ == '__main__':
    main()
