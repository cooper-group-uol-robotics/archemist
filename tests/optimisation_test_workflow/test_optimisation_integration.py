from archemist.core.processing.prcessor import WorkflowManager
from archemist.core.persistence.persistence_manager import PersistenceManager
from archemist.core.persistence.recipe_files_watchdog import RecipeFilesWatchdog
from archemist.core.persistence.yaml_handler import YamlHandler
from archemist.stations.input_station.state import InputStationPickupOp
from archemist.stations.chemspeed_flex_station.state import CSCSVJobOpDescriptor
from archemist.stations.lightbox_station.state import SampleColorOpDescriptor
from archemist.stations.output_station.state import OutputStationPlaceOp
from archemist.core.optimisation.optimisation_manager import OptimisationManager

from pathlib import Path
import time
import math

def objective_function(dyes_dict: dict) -> int:
    dye_a = dyes_dict["dye_A"][0]
    dye_b = dyes_dict["dye_B"][0]
    #  -1*round(math.sqrt(dye_a**2 + dye_b**2))
    return 100*(math.sqrt(abs(dye_b - 0.01*(dye_a**2)))+(0.01*abs(dye_a+10)))

workflow_dir = Path.cwd()#Path("tests/optimisation_test_workflow/")
server_config_file_path = workflow_dir.joinpath(f'config_files/server_settings.yaml')
server_config = YamlHandler.load_server_settings_file(server_config_file_path)

workflow_config_file_path = workflow_dir.joinpath(f'config_files/workflow_config.yaml')
recipes_dir_path = workflow_dir.joinpath(f'recipes')

# delete model file
model_file = workflow_dir.joinpath("opt_model.pickle")
if model_file.exists():
    model_file.unlink()
    
# empty recipes files
for file in recipes_dir_path.glob("*.yaml"):
    file.unlink()

db_name = server_config['db_name']

# Construct state from config file
mongo_host= server_config['mongodb_host']
persistence_mgr = PersistenceManager(mongo_host,db_name)

existing_db = False
if not existing_db:
    print('constructing new workflow state from config file')
    state = persistence_mgr.construct_state_from_config_file(workflow_config_file_path)
else:
    print('reconstructing workflow state from existing database')
    state = persistence_mgr.construct_state_from_db()

workflow_mgr = WorkflowManager(state)
recipes_watchdog = RecipeFilesWatchdog(recipes_dir_path)
recipes_watchdog.start()

opt_manager = OptimisationManager(workflow_dir, state)
time.sleep(2)
opt_manager.start_optimisation()
current_stage = "queue_recipes"

while True:
    if current_stage == "queue_recipes" and len(recipes_watchdog.recipes_queue) >= 3:
        print(f"===> {current_stage} <===")
        print("queue newly added recipe files")
        print(f"current recipe files queue count {len(recipes_watchdog.recipes_queue)}")
        for _ in range(3):
            recipe_file_path = recipes_watchdog.recipes_queue.popleft()
            recipe_dict = YamlHandler.load_recipe_file(recipe_file_path)
            workflow_mgr.queue_recipe(recipe_dict)
            print(f'new recipe with id {recipe_dict["general"]["id"]} queued')
            time.sleep(1)
        current_stage = "create_batches"
    
    if current_stage == "create_batches": 
        print(f"===> {current_stage} <===")
        print(f"current recipes queue count {len(state.recipes_queue)}")
        print("constructing new batches to attach to new recipes")
        for recipe in state.recipes_queue:
            new_batch = state.add_clean_batch()
            new_batch.attach_recipe(recipe)
            state.batches_buffer.append(new_batch)
            print(f"new clean batch ({new_batch.id}) is added")
            time.sleep(1)
        current_stage = "process_batches"

    # complete all the batches recipes
    if current_stage == "process_batches":
        print(f"===> {current_stage} <===")
        print(f"current batches buffer count {len(state.batches_buffer)}")
        print("processing batches")
        op_counter = 0
        while op_counter<12:
            for batch in state.batches_buffer:
                needed_op = batch.recipe.get_current_task_op()
                if isinstance(needed_op, InputStationPickupOp) or isinstance(needed_op, OutputStationPlaceOp):
                    needed_op.add_start_timestamp()
                    needed_op.complete_op(success=True)
                    batch.recipe.advance_state(True)
                    op_counter += 1
                elif isinstance(needed_op, CSCSVJobOpDescriptor):
                    needed_op.add_start_timestamp()
                    needed_op.complete_op(success=True)
                    for indx in range(batch.num_samples):
                        # print(needed_op.dispense_info.items())
                        sample_op = CSCSVJobOpDescriptor.from_args(dispense_info={k: [v[indx]] for k,v in 
                                                    needed_op.dispense_info.items()})
                        batch.add_station_op_to_current_sample(sample_op)
                        batch.process_current_sample()
                    batch.recipe.advance_state(True)
                    op_counter += 1
                elif isinstance(needed_op, SampleColorOpDescriptor):
                    for indx in range(batch.num_samples):
                        current_sample = batch.get_current_sample()
                        dict_vals = current_sample.extract_op_data({"CSCSVJobOpDescriptor": ["dispense_info"]})
                        sample_op = SampleColorOpDescriptor.from_args()
                        sample_op.add_start_timestamp()
                        sample_op.complete_op(success=True, red_intensity=objective_function(dict_vals), green_intensity=128, blue_intensity=128, result_filename="dummy")
                        batch.add_station_op_to_current_sample(sample_op)
                        batch.process_current_sample()
                    batch.recipe.advance_state(True)
                    op_counter += 1
                print(f"current op count: {op_counter}")
                time.sleep(1)
        current_stage = "empty_recipes_and_batches_queue"

    if current_stage == "empty_recipes_and_batches_queue":
        print(f"===> {current_stage} <===")
        print("emptying recipes queue and batches")
        for _ in range(len(state.recipes_queue)):
            state.recipes_queue.popleft()
            state.batches_buffer.popleft()
        print(f"current recipes queue count {len(state.recipes_queue)}")
        current_stage = "queue_recipes"
    
    print("====== starting a new iteration in 3 seconds ======")
    time.sleep(3)
    
    # print("batches processing is complete.")

    