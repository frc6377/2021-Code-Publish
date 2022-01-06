# Howdy Bots Quickdraw Java

[![CI](https://github.com/frc6377/2021_quickdraw/actions/workflows/main.yml/badge.svg)](https://github.com/frc6377/2021_quickdraw/actions/workflows/main.yml)

## Run Gradle commands

### Gradlew Tasks
This article aims to highlight the gradle commands supported by the WPILib team for user use. These commands can be viewed by typing `./gradlew tasks` at the root of your robot project. Not all commands shown in `./gradlew tasks` and unsupported commands will not be documented here.

### Build tasks
`./gradlew build` - Assembles and tests this project. Useful for prebuilding your project without deploying to the roboRIO. `./gradlew clean` - Deletes the build directory.

### CompileCommands tasks
`./gradlew generateCompileCommands` - Generate `compile_commands.json`. This is a configuration file that is supported by many Integrated Development Environments.

### EmbeddedTools tasks
`./gradlew deploy` - Deploy all artifacts on all targets. This will deploy your robot project to the available targets (IE, roboRIO).

`./gradlew discoverRoborio` - Determine the address(es) of target RoboRIO. This will print out the IP address of a connected roboRIO.

### GradleRIO tasks
`./gradlew downloadAll` - Download all dependencies that may be used by this project

`./gradlew $TOOL$` - Runs the tool `$TOOL$` (Replace `$TOOL$` with the name of the tool. IE, Glass, Shuffleboard, etc)

`./gradlew $TOOL$Install` - Installs the tool `$TOOL$` (Replace `$TOOL$` with the name of the tool. IE, Glass, Shuffleboard, etc)

`./gradlew InstallAllTools` - Installs all available tools. This excludes the development environment such as VSCode. It’s the users requirement to ensure the required dependencies (Java) is installed. Only recommended for advanced users!

`./gradlew riolog` - Runs a console displaying output from the default RoboRIO (roborio)

`./gradlew simulateExternalCpp` - Simulate External Task for native executable. Exports a JSON file for use by editors / tools

`./gradlew simulateExternalJava` - Simulate External Task for Java/Kotlin/JVM. Exports a JSON file for use by editors / tools

`./gradlew simulateJava` - Launches simulation for the JVM

`./gradlew vendordep` - Install vendordep JSON file from URL or local installation. See 3rd Party Libraries

For more information: https://docs.wpilib.org/pt/latest/docs/software/advanced-gradlerio/gradlew-tasks.html.

### Running Spotless
Spotless can be ran using `./gradlew spotlessApply` which will apply all formatting options. You can also specify a specific task by just adding the name of formatter. An example is `./gradlew spotlessmiscApply`.

For more details look here: https://docs.wpilib.org/pt/latest/docs/software/advanced-gradlerio/code-formatting.html.

```
./gradlew spotlessApply
```

## Config

The HowdyBot config system is designed to have compiled in Java configs for default configuration. When the robot runs
for the first time, each configuration file will be writting into the deploy folder as JSON. If someone needs to perform
a live edit of the robot configuration, they can change the files in the deploy folder and next time the robot runs, it
will pick up those changes.

Each subsystem should maintain its own config file which should be loaded each time the robot is setup.

Here is an example of a config file with two values: ConfigA and ConfigB.

```
public class TestConfig implements Jsonable {
    @JsonProperty("config_a")
    public String ConfigA = "test1";

    @JsonProperty("config_b")
    public String ConfigB = "test2";

    public TestConfig() {
    }

    public String toString() {
        return this.toJSON();
    }

    @Override
    public String toJSON() {
        ObjectMapper mapper = new ObjectMapper();
        try {
            return mapper.writeValueAsString(this);
        } catch (JsonProcessingException e) {
            return e.getMessage();
        }
    }
}
```
