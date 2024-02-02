// This file is used to define a group of targets to build in a github ci.
// In the github action, a target is defined (e.g. 'atk') which then converts into a
// list (as defined by the 'targets' variable) of targets to build.
//
// Example:
// group "atk" {
//    targets = ["dev", "chrono", "vnc"]
// }


group "atk" {
   targets = ["dev", "chrono", "vnc"]
}

target "chrono" {
   target = "chrono"
   args = {
      REMOVE_OPTIX = "true"
   }
}
