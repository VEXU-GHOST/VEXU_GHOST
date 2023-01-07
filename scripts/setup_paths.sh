echo "Adding /usr/local/lib to LD_LIBRARY_PATH in ~/.bashrc"
echo 'export LD_LIBRARY_PATH=/usr/local/lib/:$LD_LIBRARY_PATH' >> ~/.bashrc

echo "Adding ghost_sim to GAZEBO_PLUGIN_PATH in ~/.bashrc"
echo 'export GAZEBO_PLUGIN_PATH=$HOME/VEXU_GHOST/build/ghost_sim' >> ~/.bashrc