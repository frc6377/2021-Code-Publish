package frc.robot.config;

import com.fasterxml.jackson.core.JsonProcessingException;
import com.fasterxml.jackson.databind.ObjectMapper;
import edu.wpi.first.wpilibj.Filesystem;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.lang.reflect.InvocationTargetException;
import java.nio.file.Paths;

public class ConfigReader<T extends Configable> {
  // Only ever need one of these so keep a static once
  static ObjectMapper mapper = new ObjectMapper();
  Class<T> typeParameterClass;

  public ConfigReader(Class<T> typeParameterClass) {
    this.typeParameterClass = typeParameterClass;
  }

  /**
   * Read a config file from disk. If we don't have the config file, then write a default one
   *
   * @param <T> type that implements Jsonable so that we can convert it to a JSON file.
   * @param typeParameterClass a JSON encodeable class
   * @return an instance of the <code>typeParameterClass</code>
   * @throws ConfigCreationException if the <code>typeParameterClass</code> cannot be initialized
   */
  public T Read() throws ConfigCreationException {
    var cfgfile = getConfigFile(typeParameterClass.getSimpleName() + ".json");
    if (cfgfile.exists()) {
      return ReadOrCreate(cfgfile);
    }

    var defaultConfig = MakeConfig(cfgfile);
    WriteConfig(cfgfile, defaultConfig);
    return defaultConfig;
  }

  /**
   * Create a <code>File</code> object for a configuration file. If the does not yet exist create
   * the object that we can use to create the file.
   *
   * @param filename file name of the file to read or created
   * @return a file object that can be read if it exists, or created if it does not.
   */
  private File getConfigFile(String filename) {
    var deployDirectory = Filesystem.getDeployDirectory();
    for (File f : deployDirectory.listFiles()) {
      if (f.getName().equalsIgnoreCase(filename)) {
        return f;
      }
    }

    // File does not exist, so create an object to let us create one.
    var filePath = Paths.get(deployDirectory.getAbsolutePath(), filename);
    return filePath.toFile();
  }

  /**
   * Helper function for reading a file and creating it if we cannot read it.
   *
   * @param <T> type that implements Jsonable so that we can convert it to a JSON file.
   * @param cfgfile file object to read, or to create
   * @param typeParameterClass a JSON encodeable class
   * @return an instance of the <code>typeParameterClass</code>
   * @throws ConfigCreationException if the <code>typeParameterClass</code> cannot be initialized
   */
  private T ReadOrCreate(File cfgfile) throws ConfigCreationException {
    var defaultConfig = MakeConfig(cfgfile);

    try {
      var cfgWrapper = mapper.readValue(cfgfile, typeParameterClass);
      if (defaultConfig.hashCode == cfgWrapper.hashCode) {
        return cfgWrapper;
      }
      cfgfile.delete();
    } catch (IOException e) {
      // Unfortunately, but non-critical error
      return defaultConfig;
    }

    WriteConfig(cfgfile, defaultConfig);
    return defaultConfig;
  }

  /**
   * Helper function to make a new configuration file from a default Jsonable object.
   *
   * @param <T> type that implements Jsonable so that we can convert it to a JSON file.
   * @param cfgfile file object to read, or to create
   * @param typeParameterClass a JSON encodeable class
   * @return an instance of the <code>typeParameterClass</code>
   * @throws ConfigCreationException if the <code>typeParameterClass</code> cannot be initialized
   */
  private T MakeConfig(File cfgfile) throws ConfigCreationException {
    // create a default file
    T obj = null;
    try {
      obj = typeParameterClass.getDeclaredConstructor().newInstance();
    } catch (InstantiationException
        | IllegalAccessException
        | IllegalArgumentException
        | InvocationTargetException
        | NoSuchMethodException
        | SecurityException e) {
      // If something happens here, just throw our own custom exception so we have one
      // thing to track.
      throw new ConfigCreationException(
          "cannot create instance of " + typeParameterClass.getName(), e);
    }

    var hashCode = toString(obj).hashCode();
    obj.hashCode = hashCode;
    return obj;
  }

  private void WriteConfig(File cfgfile, T obj) throws ConfigCreationException {
    try {
      var json = mapper.writerWithDefaultPrettyPrinter().writeValueAsString(obj);
      WriteConfig(cfgfile, json);
    } catch (JsonProcessingException e) {
      throw new ConfigCreationException(
          "cannot convert object to JSON " + typeParameterClass.getName(), e);
    } catch (IOException e) {
      // Write an exception, but if the file can't be created that isn't a critical
      // error.
      e.printStackTrace();
    }
  }

  public String toString(T obj) {
    try {
      return mapper.writerWithDefaultPrettyPrinter().writeValueAsString(obj);
    } catch (JsonProcessingException e) {
      e.printStackTrace();
      return "";
    }
  }

  /**
   * Write a config file to disk.
   *
   * @param cfgfile file to attempt to create
   * @param json string of JSON to write
   */
  public void WriteConfig(File cfgfile, String json) throws IOException {
    cfgfile.createNewFile();
    var myWriter = new FileWriter(cfgfile);
    try {
      myWriter.write(json);
    } finally {
      myWriter.close();
    }
  }
}
