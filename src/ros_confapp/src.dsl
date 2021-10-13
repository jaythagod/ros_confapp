# ****Refer to /src/docs/docs.md usage guidelines****

#man toggle

#add name=Jude_Feature type=Concrete rel=OR to obj_dtktn
#add_feature jude to obj_dtktn 

#alter_feature nbfhv_lctm set type=Concrete rel=OR mode=static time=early status=true

#show obj_dtktn

#toggle obj_dtktn

#select obj_dtktn slam_mtplan trkn_cv pd_fc
#select obj_dtktn, slam_mtplan, trkn_cv, pd_fc

#remove ujottl_obj

#create new zero
#create_default_config zero

#load johndoe
#activate_config foobar

#show all

#ls

#config

config run

config load obj_dtktn

config unload obj_dtktn