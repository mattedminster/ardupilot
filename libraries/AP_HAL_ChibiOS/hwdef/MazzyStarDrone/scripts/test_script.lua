

function update()
    gcs:send_text(MAV_SEVERITY_INFO, 'Test script active')

    return update, RUN_INTERVAL_MS
end


