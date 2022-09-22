package org.gravity.ros.analysis.messages.ui.handler;

import java.util.List;

import org.apache.log4j.Logger;
import org.eclipse.core.commands.AbstractHandler;
import org.eclipse.core.commands.ExecutionEvent;
import org.eclipse.core.commands.ExecutionException;
import org.gravity.ros.analysis.messages.ui.ROSAnalysisUiActivator;

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
		return null;
	}
}
