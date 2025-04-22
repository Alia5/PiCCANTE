/* eslint-disable no-return-await */
import { mocks as settingMocks } from '$mocks/settings.server';
import { mocks as aboutMocks } from '$mocks/about.server';
import { mocks as statsMocks } from '$mocks/stats.server';
export const handle = async ({ event, resolve }) => {

    const mocks = {
        ...aboutMocks,
        ...settingMocks,
        ...statsMocks
    };

    // eslint-disable-next-line @typescript-eslint/no-explicit-any
    const mock = (mocks as any)?.[event.url.pathname];
    if (mock) {
        const method = event.request.method;
        const response = mock[method];
        if (response) {
            if (response.func) {
                let data;
                if (method === 'POST') {
                    data = await event.request.json();
                }
                response.func(data);
            }
            return new Response(JSON.stringify(response.body), {
                status: response.status,
                headers: {
                    'Content-Type': 'application/json'
                }
            });
        } else {
            return new Response('Method Not Allowed', { status: 405 });
        }
    }

    return await resolve(event);
};
