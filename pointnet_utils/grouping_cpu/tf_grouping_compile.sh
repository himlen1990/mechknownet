# TF1.4
g++ -std=c++11 tf_grouping.cpp -o tf_grouping_so.so -shared -fPIC -I /usr/local/lib/python2.7/dist-packages/tensorflow/include  -I /usr/local/lib/python2.7/dist-packages/tensorflow/include/external/nsync/public -L/usr/local/lib/python2.7/dist-packages/tensorflow -ltensorflow_framework -O2 -D_GLIBCXX_USE_CXX11_ABI=0
