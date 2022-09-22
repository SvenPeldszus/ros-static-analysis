package org.gravity.ros.analysis.messages.tests;

import java.io.File;
import java.util.Arrays;
import java.util.List;
import java.util.stream.Collectors;

import org.apache.log4j.BasicConfigurator;
import org.apache.log4j.Level;
import org.apache.log4j.Logger;
import org.eclipse.core.resources.IProject;
import org.eclipse.core.runtime.CoreException;
import org.eclipse.core.runtime.NullProgressMonitor;
import org.gravity.eclipse.util.EclipseProjectUtil;
import org.gravity.ros.analysis.messages.PythonProjectParser;
import org.junit.BeforeClass;
import org.junit.Test;
import org.junit.runner.RunWith;
import org.junit.runners.Parameterized;
import org.junit.runners.Parameterized.Parameters;

@RunWith(Parameterized.class)
public class ProjectParseTest {

	/**
	 * The projects that should be skipped
	 */
	private static final List<String> SKIP = Arrays.asList();

	/**
	 * The logger of this class
	 */
	protected static final Logger LOGGER = Logger.getLogger(ProjectParseTest.class);

	protected final IProject project;
	protected final String name;

	/**
	 * The constructor taking the collected projects
	 *
	 * This constructor should be only called by junit!
	 *
	 * @param name    The name of the project
	 * @param project The project
	 */
	public ProjectParseTest(final String name, final IProject project) {
		this.project = project;
		this.name = name;
	}

	@BeforeClass
	public static void initLogging() {
		// Set up logging
		BasicConfigurator.configure();
		final var rootLogger = Logger.getRootLogger();
		rootLogger.setLevel(Level.WARN);
		LOGGER.setLevel(Level.ALL);
	}

	/**
	 * The method for collecting the java projects from the workspace.
	 *
	 * This constructor should be only called by junit!
	 *
	 * @return The test parameters as needed by junit paramterized tests
	 * @throws CoreException
	 */
	@Parameters(name = "{index}: Test project: {0}")
	public static final List<Object[]> data() throws CoreException {
		LOGGER.info("Collect test data");
		final List<Object[]> projects = EclipseProjectUtil.importProjects(new File("data"), new NullProgressMonitor())
				.parallelStream().filter(project -> !SKIP.contains(project.getName()))
				.map(p -> new Object[] { p.getName(), p }).collect(Collectors.toList());
		LOGGER.info("Imported " + projects.size() + "projects into workspace.");
		return projects;
	}

	@Test(expected = NullPointerException.class)
	public void testDummy() {
		new PythonProjectParser().parse();
	}
}