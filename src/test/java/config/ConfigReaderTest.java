package config;

import static org.junit.Assert.*;

import edu.wpi.first.wpilibj.Filesystem;
import frc.robot.config.ConfigCreationException;
import frc.robot.config.ConfigReader;
import java.io.File;
import java.io.IOException;
import java.nio.file.Paths;
import org.junit.After;
import org.junit.Before;
import org.junit.Test;

public class ConfigReaderTest {
  private File testCfgFile = null;
  private ConfigReader<TestConfig> reader = new ConfigReader<TestConfig>(TestConfig.class);

  @Before
  public void setup() {
    testCfgFile =
        Paths.get(Filesystem.getDeployDirectory().getAbsolutePath(), "TestConfig.json").toFile();
    cleanup(); // In case we exited in the middle of a test.
  }

  @After
  public void cleanup() {
    testCfgFile.delete();
  }

  /** Test that we'll write a file and return default values when no file exists. */
  @Test
  public void objectNoFileDefault() throws ConfigCreationException {
    TestConfig cfg = reader.Read();
    validateDefault(cfg);
  }

  /** Test that when the file exists, we will read from it instead of using the default. */
  @Test
  public void objectReadFile() throws ConfigCreationException, IOException {
    var testConfig = new TestConfig();
    // Need to get the hash code right or the reader won't use this string.
    var hashCode = reader.toString(testConfig).hashCode();
    testConfig.ConfigA = "file_test_for_a";
    testConfig.ConfigB = "file_test_for_b";
    testConfig.hashCode = hashCode;
    reader.WriteConfig(testCfgFile, reader.toString(testConfig));

    TestConfig cfg = reader.Read();

    assertEquals(cfg.hashCode, hashCode);
    assertEquals(cfg.ConfigA, "file_test_for_a");
    assertEquals(cfg.ConfigB, "file_test_for_b");
    assertTrue(testCfgFile.exists());
  }

  @Test
  public void objectReadFileBadHash() throws ConfigCreationException, IOException {
    var testConfig = new TestConfig();
    testConfig.ConfigA = "file_test_for_a";
    testConfig.ConfigB = "file_test_for_b";
    testConfig.hashCode = 0;
    reader.WriteConfig(testCfgFile, reader.toString(testConfig));

    TestConfig cfg = reader.Read();

    assertNotEquals(cfg.hashCode, 0);
    assertEquals(cfg.ConfigA, "test1");
    assertEquals(cfg.ConfigB, "test2");
  }

  /**
   * Test that use the default values when we don't have JSON The message in the file is not valid
   * JSON.
   */
  @Test
  public void canHandleNonJSONFile() throws ConfigCreationException, IOException {
    reader.WriteConfig(testCfgFile, "this is not JSON");

    TestConfig cfg = reader.Read();
    validateDefault(cfg);
  }

  /**
   * Test that use the default values when we badly formatted JSON In this test, we use
   * {"config_a"="invalid_entry"}. The equal sign is a JSON format error.
   */
  @Test
  public void canHandleInvalidJSONFormat() throws ConfigCreationException, IOException {
    reader.WriteConfig(
        testCfgFile, "{\"config_a\"= \"invalid_entry\", \"config_b\": \"this_test_b\"}");

    TestConfig cfg = reader.Read();
    validateDefault(cfg);
  }

  /**
   * Test that use the default values when we have correct JSON that can't be loaded. In this test,
   * the value `config` is added instead of `config_a`
   */
  @Test
  public void canHandleInvalidJSONData() throws ConfigCreationException, IOException {
    reader.WriteConfig(testCfgFile, "{\"config\": \"test1\", \"config_b\": \"this_test_b\"}");

    TestConfig cfg = reader.Read();
    validateDefault(cfg);
  }

  private void validateDefault(TestConfig cfg) {
    assertEquals(cfg.ConfigA, "test1");
    assertEquals(cfg.ConfigB, "test2");
    assertTrue(testCfgFile.exists());
  }
}
