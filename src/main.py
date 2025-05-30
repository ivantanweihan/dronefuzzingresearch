from ubuntu_handler import Handler
# from macos_handler import Handler

# Clear any background processes
# The first argument is the policy name and will be checked against the dictionary 
# scenario_procedures in pymavlink_simulation which will call the function defined
# declared in the dictionary
handler1 = Handler('A.ALT_HOLD1', algo='rand', ranking=False, k=0) # choose between rand, drl, and ga
handler1.kill_all()
for i in range(1):
    handler1.helper.new_episode(i)
    handler1.start_simulation()