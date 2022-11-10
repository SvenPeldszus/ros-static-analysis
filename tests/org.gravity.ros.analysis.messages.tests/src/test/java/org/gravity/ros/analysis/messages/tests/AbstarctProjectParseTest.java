package org.gravity.ros.analysis.messages.tests;

import java.io.File;
import java.util.List;
import java.util.stream.Collectors;

import org.apache.log4j.BasicConfigurator;
import org.apache.log4j.Level;
import org.apache.log4j.Logger;
import org.eclipse.core.resources.IProject;
import org.eclipse.core.runtime.CoreException;
import org.eclipse.core.runtime.NullProgressMonitor;
import org.gravity.eclipse.util.EclipseProjectUtil;
import org.junit.BeforeClass;
import org.python.pydev.parser.jython.SimpleNode;
import org.python.pydev.shared_core.parsing.BaseParser.ParseOutput;

public abstract class AbstarctProjectParseTest {

	/**
	 * The test project
	 */
	protected final IProject project;
	
	/**
	 * The logger of this class
	 */
	private static final Logger LOGGER = Logger.getLogger(AbstarctProjectParseTest.class);

	/**
	 * Initializes the test with the project of the given name from the data folder
	 * 
	 * @param name The name of the project
	 * @throws CoreException If the project cannot be imported 
	 */
	protected AbstarctProjectParseTest(String name) throws CoreException {
		LOGGER.info("Collect test data");
		final List<Object[]> projects = EclipseProjectUtil.importProjects(new File("data"), new NullProgressMonitor())
				.parallelStream()
				.map(p -> new Object[] { p.getName(), p }).collect(Collectors.toList());
		LOGGER.info("Imported " + projects.size() + "projects into workspace.");
		project =  EclipseProjectUtil.getProjectByName(name);
	}
	
	
	@BeforeClass
	public static void initLogging() {
		// Set up logging
		BasicConfigurator.configure();
		final var rootLogger = Logger.getRootLogger();
		rootLogger.setLevel(Level.WARN);
		LOGGER.setLevel(Level.ALL);
	}

	public void parseContentPrint(List<ParseOutput> parsedList) {
		for (ParseOutput parseOutput : parsedList) {
			SimpleNode ast = (SimpleNode) parseOutput.ast;
			System.out.println(ast.toString());
		}
	}
}


