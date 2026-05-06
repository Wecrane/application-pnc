你现在接手 Apollo EDU PnC 仓库，路径是：

/home/skye/application-pnc

日志位置：

1. 本地 Apollo planning 日志：
   /home/skye/application-pnc/data/log/

   重点看：
   data/log/planning.log.INFO.*

   不要整文件读取，文件很大。用 rg / sed / tail 按关键词和行号查。
2. 评测系统给出的日志：
   /home/skye/下载/log/

   如果我说“评测日志已更新”，就优先看这里；如果我说“评测日志不用看”，就只看 data/log/ 里的本地 planning 日志。

不要编译，等我手动编译

可参考planning-module-navigator这个skills

下面是需求：
