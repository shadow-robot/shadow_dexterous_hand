# Stage 1
FROM public.ecr.aws/shadowrobot/build-tools:focal-noetic AS builder

ENV PROJECTS_WS=/home/user/projects/shadow_robot
ENV rosinstall_repo=shadow_dexterous_hand
ENV rosinstall_repo_branch=noetic-devel
ENV aurora_branch="master"
ENV aurora_script="https://raw.githubusercontent.com/shadow-robot/aurora/$aurora_branch/bin/run-ansible.sh"
ENV remote_shell_script="https://raw.githubusercontent.com/shadow-robot/sr-build-tools/master/ansible/deploy.sh"

# Install everything needed to build the project
RUN apt-get update && apt-get install -y --no-install-recommends \
    gosu \
    && rm -rf /var/lib/apt/lists/*

# Run the deployment script (installs all dependencies)
RUN wget -O /tmp/oneliner "$remote_shell_script" && \
    chmod +x /tmp/oneliner && \
    gosu $MY_USERNAME /tmp/oneliner -w $PROJECTS_WS/base -r $rosinstall_repo -b $rosinstall_repo_branch -i repository.rosinstall -v "noetic" -s false -t pyqtgraph

# Install software via aurora
RUN wget -O /tmp/aurora "$aurora_script" && \
    chmod +x /tmp/aurora && \
    gosu $MY_USERNAME /tmp/aurora install_software --branch $aurora_branch software=[production_tools,aws-cli,libglvnd,vscode,warehouse_ros]

# Stage 2
FROM ros:noetic-robot AS runtime

# Copy files from builder
COPY --from=builder /home/user/projects/shadow_robot /home/user/projects/shadow_robot
COPY --from=builder /usr/local /usr/local
COPY --from=builder /opt/ros /opt/ros

# Add terminator back into runtime
RUN apt-get update && apt-get install -y --no-install-recommends \
    terminator \
    && rm -rf /var/lib/apt/lists/*

# Set entrypoint
ENTRYPOINT ["/usr/local/bin/entrypoint.sh"]
CMD ["/usr/bin/terminator"]