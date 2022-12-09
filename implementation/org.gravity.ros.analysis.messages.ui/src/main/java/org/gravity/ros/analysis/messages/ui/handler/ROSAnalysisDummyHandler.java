package org.gravity.ros.analysis.messages.ui.handler;

import java.util.HashSet;
import java.util.List;
import java.util.Set;

import org.apache.log4j.Logger;
import org.eclipse.core.commands.AbstractHandler;
import org.eclipse.core.commands.ExecutionEvent;
import org.eclipse.core.commands.ExecutionException;
import org.eclipse.core.resources.IProject;
import org.eclipse.core.runtime.CoreException;
import org.gravity.ros.analysis.messages.PythonProjectParser;
import org.gravity.ros.analysis.messages.ui.ROSAnalysisUiActivator;
import org.python.pydev.navigator.elements.PythonSourceFolder;

/**
 * A dummy handler in registered in the plugin.xml in the package explorer
 *
 * @author speldszus
 *
 */
public class ROSAnalysisDummyHandler extends AbstractHandler {

	private static final Logger LOGGER = Logger.getLogger(ROSAnalysisDummyHandler.class);

	@Override
	public Object execute(ExecutionEvent event) throws ExecutionException {
		List<?> selection = ROSAnalysisUiActivator.getSelection(event);
		LOGGER.info("The current selection in the package explorer is:");
		selection.forEach(LOGGER::info);
		//TODO: Do something with the selected elements
		Set<IProject> projects = new HashSet<>();
		for(Object obj: selection) {
			if(obj instanceof IProject)
				projects.add((IProject) obj);
			else if (obj instanceof PythonSourceFolder) {
				projects.add((IProject) ((PythonSourceFolder) obj).getActualObject());
			}
		}
		
		for(IProject project : projects) {
			try {
				new PythonProjectParser().parse(project);
			} catch (CoreException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			}
		}
		return null;
	}
}
