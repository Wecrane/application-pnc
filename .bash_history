aem bootstrap start --plus
aem profile use default
buildtool build -p modules/planning/ -j15
aem profile use default
cat >> ~/.bashrc << 'EOF'

# === 持久化 bash 历史（跨容器重启保留）===
export HISTFILE=/apollo_workspace/.bash_history
export HISTSIZE=10000
export HISTFILESIZE=20000
export HISTCONTROL=ignoredups:erasedups
shopt -s histappend
PROMPT_COMMAND="history -a; history -c; history -r; $PROMPT_COMMAND"
EOF

source ~/.bashrc
aem bootstrap start --plus
buildtool build -p modules/planning/ -j15
aem profile use default
buildtool build -p modules/planning/ -j15
aem profile use default
buildtool build -p modules/planning/ -j15
aem profile use default
buildtool build -p modules/planning/ -j15
aem profile use default
buildtool build -p modules/planning/ -j15
aem profile use default
cyber_launch start modules/planning/planning_component/launch/planning.launch
aem profile use default && cyber_launch start modules/planning/planning_component/launch/planning.launch
buildtool build -p modules/planning/ -j15
aem profile use default
buildtool build -p modules/planning/ -j15
aem profile use default
buildtool build -p modules/planning/ -j15
aem profile use default
buildtool build -p modules/planning/ -j15
aem profile use default
buildtool build -p modules/planning/ -j15
aem profile use default
buildtool build -p modules/planning/ -j15
aem profile use default
buildtool build -p modules/planning/ -j15
aem profile use default
buildtool build -p modules/planning/ -j15
aem profile use default
buildtool build -p modules/planning/ -j15
aem profile use default
buildtool build -p modules/planning/ -j15
aem profile use default
buildtool build -p modules/planning/ -j15
aem profile use default
buildtool build -p modules/planning/ -j15
aem profile use default
buildtool build -p modules/planning/ -j15
aem profile use default
buildtool build -p modules/planning/ -j15
aem profile use default
buildtool build -p modules/planning/ -j15
aem profile use default
buildtool build -p modules/planning/ -j15
aem profile use default
buildtool build -p modules/planning/ -j15
aem profile use default
buildtool build -p modules/planning/ -j15
aem profile use default
aem bootstrap start --plus
buildtool build -p modules/planning/ -j15
aem profile use default
buildtool build -p modules/planning/ -j15
aem profile use default
buildtool build -p modules/planning/ -j15
aem profile use default
buildtool build -p modules/planning/ -j15
aem profile use default
buildtool build -p modules/planning/ -j15
aem profile use default
buildtool build -p modules/planning/ -j15
aem profile use default
buildtool build -p modules/planning/ -j15
aem profile use default
buildtool build -p modules/planning/ -j15
aem profile use default
buildtool build -p modules/planning/ -j15
aem profile use default
buildtool build -p modules/planning/ -j15
aem profile use default
buildtool build -p modules/planning/ -j15
aem profile use default
buildtool build -p modules/planning/ -j15
aem profile use default
buildtool build -p modules/planning/ -j15
aem profile use default
buildtool build -p modules/planning/ -j15
aem profile use default
buildtool build -p modules/planning/ -j15
aem profile use default
buildtool build -p modules/planning/ -j15
aem profile use default
buildtool build -p modules/planning/ -j15
aem profile use default
buildtool build -p modules/planning/ -j15
aem profile use default
buildtool build -p modules/planning/ -j15
aem profile use default
buildtool build -p modules/planning/ -j15
aem profile use default
buildtool build -p modules/planning/ -j15
aem profile use default
buildtool build -p modules/planning/ -j15
aem profile use default
buildtool build -p modules/planning/ -j15
aem profile use default
aem profile use sample
buildtool build -p modules/planning/ -j15
buildtool build -p core -j15
aem bootstrap start --plus
aem bootstrap restart --plus
buildtool profile config init --package planning --profile=default
aem profile use default
aem bootstrap restart --plus
