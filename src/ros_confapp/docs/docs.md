# Library Documentation

## Setup

In the parent directory run the command **pipenv install**

<div class='guide'>

-Description:
-Usage:

</div>

<div class='features'>

- Description: A feature is realised as a key value pair object with its properties stores in its 
 corresponding props.json file, referenced by the id of the feature. Sub features are represented
 by a nested array of objects with a "sub" key.

- Structure: {
                "id":"abc",
                "name": "xxxxx",
                "sub":[{
                        "id":"efg",
                        "name": "yyyyy"
                        }]
            }

- Properties:

- Preconditions:
    1. Abstract features by default have a binding time of early and binding mode of static
    2. Abstract features cannot be deactivated or unloaded
    3. Alternative features by default have an XOR relationship

</div>


##### Basic Commands

**toggle:**

<div class='toggle'>

- Syntax : toggle feature_id

- Description: The toggle command deactivates a feature that is already activated and vise versa.

- Usage:

</div>

**add:**

<div class='add'>

- Syntax : add_feature feature_name to feature_id

- Description: The add_feature command creates a new feature. The command sysntax allows the user to 
  add the  the feature to a specific point in the feature model.

- Usage:

</div>

**remove:**

<div class='remove'>

- Syntax : remove feature_id

- Description: The remove command removes a feature from the feature model.

- Usage:

</div>



<div class='man'>

- Syntax : man command_name, man all

- Description: The "man" command provides a concise and easily accessible user manual 
 interface that serves as a quick look-up point for command language syntax, descriptions 
 and allowed parameters.

- Usage Examples: 
1. man toggle : This command returns all information in the dsl documentation 
   about the toggle command.

2. man all : Returns the entire contents of the user manual.

</div>

<div class='show'>

- Syntax : show feature_id, show all

- Description: The "show" command returns a hierarchical breakdown of either a 
  specified feature or the entire feature model.

- Usage Examples: 
1. show top_ctrl : This command returns the feature "top_ctrl" together with 
   all its sub-features.

2. show all : Returns all the features in the currently active feature model.

</div>

**select:**

<div class='select'>

- Syntax : select feature_id1 feature_id2 feature_id3

- Description: The select command enables the bulk selection or activation of features in the feature model.
  The list of ids of the features to be selected are separated by a single space after the select command.

- Usage:

</div>